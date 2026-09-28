#!/usr/bin/env python3
"""Render readable Mermaid architecture views from static inventory + live ROS 2 runtime.

Outputs (all in --output-dir):

* ``runtime-overview.mmd`` — container map: Pi -> Docker container, one box per
  container listing its ROS nodes, edges aggregated per container pair and
  labelled with the topics that flow between them.
* ``runtime-graph.mmd``    — full node-level graph with infrastructure noise
  (``/tf``, ``/diagnostics``, tf2 listener helpers, ...) removed and 1:1 topics
  folded into labelled edges.
* ``runtime-graph.md``     — Markdown report for the job summary: container map,
  one small diagram per container, placement evidence table, hidden noise.

Physical placement (which container runs a node) is resolved from repository
evidence only: compose ``command`` -> start script -> ``ros2 run`` /
``ros2 launch`` / ``-r __node:=`` -> launch files / setup.py entry points.
Every placement carries its evidence; nodes without evidence stay "unplaced".
"""

from __future__ import annotations

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path

# Topics every ROS node touches; drawing them turns the graph into a hairball.
INFRA_TOPICS = {
    "/tf",
    "/tf_static",
    "/diagnostics",
    "/rosout",
    "/parameter_events",
    "/clock",
    "/robot_description",
    "/bond",
}
INFRA_TOPIC_SUFFIXES = ("/transition_event",)

# Helper nodes created implicitly by libraries, not by our launch files.
HELPER_NODE_PATTERNS = (
    re.compile(r"(^|/)transform_listener_impl_[0-9a-f]+$"),  # tf2_ros::TransformListener
    re.compile(r"(^|/)launch_ros_\d+$"),  # ros2 launch process itself
    re.compile(r"(^|/)_ros2cli_(daemon_)?\d+(_[0-9a-f]+)?$"),  # ros2 CLI + its daemon (the audit itself)
    re.compile(r"_rclcpp_node$"),  # nav2 BT action helper nodes
)

# Nodes hosted inside another node's process by upstream packages.
UPSTREAM_COMPANIONS = {
    "global_costmap/global_costmap": ("planner_server", "nav2 planner_server hosts the global costmap"),
    "local_costmap/local_costmap": ("controller_server", "nav2 controller_server hosts the local costmap"),
    "diff_drive_controller": ("controller_manager", "controller loaded by ros2_control controller_manager"),
    "joint_state_broadcaster": ("controller_manager", "controller loaded by ros2_control controller_manager"),
}

# Default node names of upstream executables whose name differs from the executable.
UPSTREAM_EXECUTABLE_NODES = {
    "ros2_control_node": "controller_manager",
    "usb_cam_node_exe": "usb_cam",
}

HOST_BY_COMPOSE_DIR = {"docker/main/": "Main Pi", "docker/vision/": "Vision Pi"}
DEFAULT_HOST_ADDRESSES = {"Main Pi": "10.1.1.20", "Vision Pi": "10.1.1.21"}

MAX_EDGE_TOPICS = 3
MAX_BOX_NODES = 8


# --------------------------------------------------------------------------- helpers


def norm(value):
    return re.sub(r"[^a-z0-9]", "", (value or "").lower())


def canonical_node(value):
    """Normalize ROS node names while preserving namespaces."""
    return (value or "").strip().lstrip("/")


def leaf(node):
    return node.rsplit("/", 1)[-1]


def safe(value):
    return (
        str(value)
        .replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("|", "&#124;")
    )


def mid(prefix, value):
    """Stable Mermaid identifier."""
    return prefix + "_" + re.sub(r"[^A-Za-z0-9]", "_", value).strip("_")


def is_infra_topic(topic):
    return topic in INFRA_TOPICS or topic.endswith(INFRA_TOPIC_SUFFIXES)


def is_helper_node(node):
    return any(p.search(node) for p in HELPER_NODE_PATTERNS)


def node_class_map(inventory):
    by_norm = defaultdict(list)
    for item in inventory.get("classes", []):
        if not item.get("is_node") or not item.get("name"):
            continue
        name = item["name"]
        key = norm(name)
        for candidate in {key, re.sub(r"node$", "", key)}:
            if candidate:
                by_norm[candidate].append(name)
    return {k: sorted(set(v))[0] for k, v in by_norm.items() if len(set(v)) == 1}


def resolve_class(node, classes):
    key = norm(leaf(node))
    return classes.get(key) or classes.get(re.sub(r"node$", "", key))


# --------------------------------------------------------------------------- runtime


def endpoint_node(endpoint, runtime_nodes):
    """Map a ``ros2 topic info --verbose`` endpoint to a captured runtime node."""
    name = canonical_node(endpoint.get("node"))
    if not name:
        return None
    namespace = canonical_node(endpoint.get("namespace"))
    candidates = [f"{namespace}/{name}"] if namespace else []
    candidates.append(name)
    for candidate in candidates:
        if candidate in runtime_nodes:
            return candidate
    matches = [n for n in runtime_nodes if leaf(n) == name]
    if len(matches) == 1:
        return matches[0]
    return candidates[0]


def topic_edges(runtime):
    """Return {topic: (publishers, subscribers)} using runtime node identities."""
    runtime_nodes = {canonical_node(x) for x in runtime.get("nodes", []) if x}
    result = {}
    for topic, info in runtime.get("topic_info", {}).items():
        pubs = {endpoint_node(x, runtime_nodes) for x in info.get("publishers", []) or []}
        subs = {endpoint_node(x, runtime_nodes) for x in info.get("subscribers", []) or []}
        pubs.discard(None)
        subs.discard(None)
        result[topic] = (pubs, subs)
    return result


def connected_topics(runtime, include_system=False):
    """Topics with at least one publisher and one subscriber, busiest first."""
    edges = topic_edges(runtime)
    result = [
        t
        for t in runtime.get("topics", [])
        if (include_system or t not in INFRA_TOPICS) and edges.get(t, (set(), set()))[0] and edges[t][1]
    ]
    return sorted(result, key=lambda t: (-(len(edges[t][0]) + len(edges[t][1])), t))


# --------------------------------------------------------------------------- placement


def compose_services(inventory):
    services = []
    for group in inventory.get("containers", []):
        source = group.get("file", "")
        host = next((h for prefix, h in HOST_BY_COMPOSE_DIR.items() if prefix in source), "Other")
        for service in group.get("services", []):
            item = dict(service)
            item["host"] = host
            item["source"] = source
            services.append(item)
    return services


def service_label(service):
    return service.get("container_name") or service.get("name") or "container"


def _as_text(value):
    if not value:
        return ""
    if isinstance(value, (list, tuple)):
        return " ".join(str(x) for x in value)
    return str(value)


class RepoIndex:
    """Lazy lookup of launch files and Python entry points in the checkout."""

    def __init__(self, root):
        self.root = Path(root)
        self._launch = None
        self._entry_points = None

    def rel(self, path):
        try:
            return str(Path(path).relative_to(self.root))
        except ValueError:
            return str(path)

    def launch_files(self, basename):
        if self._launch is None:
            self._launch = defaultdict(list)
            for pattern in ("*launch.py", "*launch.xml"):
                for path in self.root.rglob(pattern):
                    parts = set(path.parts)
                    if {".git", "node_modules", "build", "install"} & parts:
                        continue
                    self._launch[path.name].append(path)
        return sorted(self._launch.get(basename, []))

    def entry_point(self, executable):
        if self._entry_points is None:
            self._entry_points = {}
            for setup in (self.root / "src").glob("*/setup.py"):
                text = setup.read_text(encoding="utf-8", errors="ignore")
                for exe, module in re.findall(r"['\"]\s*([\w.-]+)\s*=\s*([\w.]+):\w+\s*['\"]", text):
                    module_path = setup.parent / (module.replace(".", "/") + ".py")
                    self._entry_points.setdefault(exe, module_path)
        return self._entry_points.get(executable)


NODE_NAME_IN_CODE = re.compile(r"(?:super\(\)\.__init__|Node\.__init__\(\s*self\s*,|\bNode)\(\s*['\"]([\w/]+)['\"]")


def names_from_python_module(path):
    if not path or not Path(path).is_file():
        return []
    return NODE_NAME_IN_CODE.findall(Path(path).read_text(encoding="utf-8", errors="ignore"))


def names_from_executable(executable, repo):
    """Node names an executable is known to create, with evidence."""
    exe = executable.removesuffix(".py")
    found = []
    module = repo.entry_point(exe) or repo.entry_point(executable)
    for name in names_from_python_module(module):
        found.append((name, f"{repo.rel(module)} creates node '{name}'"))
    if exe in UPSTREAM_EXECUTABLE_NODES:
        found.append((UPSTREAM_EXECUTABLE_NODES[exe], f"upstream default name of {exe}"))
    found.append((exe, f"default node name of executable {exe}"))
    return found


LAUNCH_NAME = re.compile(r"\bname\s*=\s*['\"]([\w/]+)['\"]")
LAUNCH_EXECUTABLE = re.compile(r"\bexecutable\s*=\s*['\"]([\w./-]+)['\"]")


def names_from_launch(basename, repo, host_prefix):
    found = []
    files = repo.launch_files(basename)
    preferred = [p for p in files if repo.rel(p).startswith(host_prefix)]
    for path in preferred or files:
        text = path.read_text(encoding="utf-8", errors="ignore")
        rel = repo.rel(path)
        for name in LAUNCH_NAME.findall(text):
            found.append((name, f"{rel} declares name='{name}'"))
        for exe in LAUNCH_EXECUTABLE.findall(text):
            for name, why in names_from_executable(exe, repo):
                found.append((name, f"{rel} runs {exe}; {why}"))
    return found


def start_script(service, repo, host_prefix):
    text = " ".join([_as_text(service.get("entrypoint")), _as_text(service.get("command"))])
    for script in re.findall(r"(start_[\w-]+\.sh)", text):
        hits = sorted((repo.root / host_prefix).rglob(script)) if (repo.root / host_prefix).is_dir() else []
        if hits:
            return hits[0]
    return None


def service_node_claims(service, repo):
    """(node_name, evidence) pairs a compose service is proven to start."""
    host_prefix = next((p for p in HOST_BY_COMPOSE_DIR if p in service.get("source", "")), "")
    texts = [("compose command", " ".join([_as_text(service.get("entrypoint")), _as_text(service.get("command"))]))]
    script = start_script(service, repo, host_prefix)
    if script:
        texts.append((repo.rel(script), script.read_text(encoding="utf-8", errors="ignore")))
    claims = []
    for origin, text in texts:
        for name in re.findall(r"__node:=([\w/]+)", text):
            claims.append((name, f"{origin}: -r __node:={name}"))
        for pkg, exe in re.findall(r"ros2\s+run\s+([\w-]+)\s+([\w.-]+)", text):
            for name, why in names_from_executable(exe, repo):
                claims.append((name, f"{origin}: ros2 run {pkg} {exe}; {why}"))
        for launch in re.findall(r"([\w/.-]*launch\.(?:py|xml))", text):
            for name, why in names_from_launch(Path(launch).name, repo, host_prefix):
                claims.append((name, f"{origin}: ros2 launch {Path(launch).name}; {why}"))
    return claims


def resolve_placement(nodes, inventory, repo_root):
    """Map runtime nodes to compose services: {node: {"service", "evidence", "method"}}."""
    repo = RepoIndex(repo_root)
    services = [s for s in compose_services(inventory) if s["host"] != "Other"]
    by_name = defaultdict(list)
    for service in services:
        for name, why in service_node_claims(service, repo):
            by_name[leaf(name)].append((service, why))

    placement = {}
    for node in nodes:
        claims = by_name.get(leaf(node), [])
        owners = {id(s): (s, why) for s, why in claims}
        if len(owners) == 1:
            service, why = next(iter(owners.values()))
            placement[node] = {"service": service, "evidence": why, "method": "start-script"}

    # Library-hosted nodes follow their host node.
    for node in nodes:
        if node in placement or node not in UPSTREAM_COMPANIONS:
            continue
        parent, why = UPSTREAM_COMPANIONS[node]
        if parent in placement:
            placement[node] = {"service": placement[parent]["service"], "evidence": why, "method": "companion"}

    # Namespaced nodes whose namespace equals a container name (e.g. /rtabmap/*).
    for node in nodes:
        if node in placement or "/" not in node:
            continue
        ns = norm(node.split("/", 1)[0])
        matches = [s for s in services if ns and ns in {norm(s.get("name")), norm(s.get("container_name"))}]
        if len(matches) == 1:
            placement[node] = {
                "service": matches[0],
                "evidence": f"namespace /{node.split('/', 1)[0]} equals container {service_label(matches[0])}",
                "method": "namespace",
            }

    # Last resort: a unique container whose name contains the node name (or vice versa).
    for node in nodes:
        if node in placement:
            continue
        key = norm(leaf(node))
        matches = []
        for service in services:
            names = {norm(service.get("name")), norm(service.get("container_name"))} - {""}
            if any(len(n) >= 4 and (n in key or key in n) for n in names):
                matches.append(service)
        if len(matches) == 1:
            placement[node] = {
                "service": matches[0],
                "evidence": f"name match only: {leaf(node)} ~ {service_label(matches[0])}",
                "method": "name-match",
            }
    return placement


# --------------------------------------------------------------------------- model


class Model:
    def __init__(self, runtime, inventory, repo_root):
        all_nodes = sorted({canonical_node(x) for x in runtime.get("nodes", []) if x})
        self.edges = topic_edges(runtime)
        for pubs, subs in self.edges.values():
            all_nodes = sorted(set(all_nodes) | pubs | subs)
        self.helper_nodes = [n for n in all_nodes if is_helper_node(n)]
        self.nodes = [n for n in all_nodes if not is_helper_node(n)]
        self.classes = node_class_map(inventory)
        self.placement = resolve_placement(self.nodes, inventory, repo_root)

        self.hosts = dict(DEFAULT_HOST_ADDRESSES)
        for capture in runtime.get("captures", []):
            if capture.get("pi") and capture.get("host"):
                self.hosts[capture["pi"]] = capture["host"]

        visible = set(self.nodes)
        self.infra_topics = {}
        self.topics = {}
        self.dangling = []
        for topic in sorted(self.edges):
            pubs, subs = self.edges[topic]
            if is_infra_topic(topic):
                if pubs or subs:
                    self.infra_topics[topic] = (len(pubs), len(subs))
                continue
            pubs, subs = pubs & visible, subs & visible
            if pubs and subs:
                self.topics[topic] = (pubs, subs)
            elif pubs or subs:
                self.dangling.append(topic)

    def container_of(self, node):
        meta = self.placement.get(node)
        if not meta:
            return None
        return (meta["service"]["host"], service_label(meta["service"]))

    def containers(self):
        grouped = defaultdict(list)
        for node in self.nodes:
            grouped[self.container_of(node)].append(node)
        return grouped

    def container_edges(self):
        """{(src_container, dst_container): sorted topics} across container boundaries."""
        result = defaultdict(set)
        for topic, (pubs, subs) in self.topics.items():
            for pub in pubs:
                for sub in subs:
                    src, dst = self.container_of(pub), self.container_of(sub)
                    if src != dst:
                        result[(src, dst)].add(topic)
        return {k: sorted(v) for k, v in result.items()}


UNPLACED = ("Unplaced", "no placement evidence")


def container_id(container):
    host, name = container or UNPLACED
    return mid("C", f"{host}_{name}")


def host_id(host):
    return mid("H", host)


def topic_list_label(topics, limit=MAX_EDGE_TOPICS):
    shown = topics[:limit]
    label = "<br/>".join(safe(t) for t in shown)
    if len(topics) > limit:
        label += f"<br/>+{len(topics) - limit} more"
    return label


HEADER = [
    '%%{init: {"theme":"base","themeVariables":{"fontSize":"14px"},'
    '"flowchart":{"curve":"basis","nodeSpacing":30,"rankSpacing":60,"htmlLabels":true}}}%%',
]

CLASS_DEFS = [
    "    classDef container fill:#eef6ff,stroke:#3b6fd8,color:#0f2a55,stroke-width:1.5px,text-align:left;",
    "    classDef unplaced fill:#fff7ed,stroke:#d97706,color:#7c2d12,stroke-width:1.5px,stroke-dasharray:4 3;",
    "    classDef rosnode fill:#eef6ff,stroke:#3b6fd8,color:#0f2a55,stroke-width:1.2px;",
    "    classDef external fill:#f3f4f6,stroke:#9ca3af,color:#374151,stroke-dasharray:3 3;",
    "    classDef topic fill:#ecfdf5,stroke:#16a34a,color:#14532d;",
]


def host_order(model, grouped):
    hosts = [h for h in model.hosts if any(c and c[0] == h for c in grouped)]
    return hosts


HOST_CLASSES = ["hostA", "hostB", "hostC"]
HOST_CLASS_DEFS = [
    "    classDef hostA fill:#e0ecff,stroke:#2f5fd0,color:#0b2350,stroke-width:1.5px;",
    "    classDef hostB fill:#f1e8ff,stroke:#7c3aed,color:#2e1065,stroke-width:1.5px;",
    "    classDef hostC fill:#e6f7f1,stroke:#0f9f6e,color:#063b29,stroke-width:1.5px;",
]


HOST_SUBGRAPH_STYLES = {
    "hostA": "fill:#f3f7ff,stroke:#2f5fd0,stroke-width:1.5px",
    "hostB": "fill:#faf5ff,stroke:#7c3aed,stroke-width:1.5px",
    "hostC": "fill:#f0fbf7,stroke:#0f9f6e,stroke-width:1.5px",
}


def host_class(model, host):
    hosts = list(model.hosts)
    index = hosts.index(host) if host in hosts else len(hosts)
    return HOST_CLASSES[index % len(HOST_CLASSES)]


def render_overview(model):
    """Container map. Hosts are colours, not boxes: boxes force cross-Pi edges around the frame."""
    grouped = model.containers()
    lines = ["%% Auto-generated by tools/architecture_runtime_graph.py — container map", *HEADER, "flowchart LR"]
    legend = []
    for host in host_order(model, grouped):
        legend.append(
            f'    {host_id(host)}["🖥️ <b>{safe(host)}</b><br/>{safe(model.hosts[host])}"]:::{host_class(model, host)}'
        )
        for container in sorted(c for c in grouped if c and c[0] == host):
            nodes = sorted(grouped[container])
            body = "<br/>".join(safe(n) for n in nodes[:MAX_BOX_NODES])
            if len(nodes) > MAX_BOX_NODES:
                body += f"<br/>+{len(nodes) - MAX_BOX_NODES} more"
            lines.append(
                f'    {container_id(container)}["<b>🐳 {safe(container[1])}</b><br/><small>{body}</small>"]'
                f":::{host_class(model, host)}"
            )
    if None in grouped:
        body = "<br/>".join(safe(n) for n in sorted(grouped[None]))
        lines.append(f'    {container_id(None)}["<b>❓ container not proven</b><br/><small>{body}</small>"]:::unplaced')
    for (src, dst), topics in sorted(
        model.container_edges().items(), key=lambda x: (container_id(x[0][0]), container_id(x[0][1]))
    ):
        lines.append(f'    {container_id(src)} -->|"{topic_list_label(topics)}"| {container_id(dst)}')
    if legend:
        lines.append('    subgraph Legend["Legend: container colour = Pi"]')
        lines.append("        direction TB")
        lines += ["    " + x for x in legend]
        lines.append("    end")
    lines += ["", *CLASS_DEFS, *HOST_CLASS_DEFS]
    return "\n".join(lines) + "\n"


def topic_flow(topics, node_id, home):
    """Split topics into pill nodes (grouped by home container) and edges.

    A topic with exactly one publisher and one subscriber is folded into a labelled
    edge; a shared topic becomes a pill placed inside ``home(publishers)`` so that
    its edges stay short.
    """
    pills = defaultdict(list)
    edges = []
    folded = defaultdict(list)
    for topic in sorted(topics):
        pubs, subs = topics[topic]
        pub_ids = list(dict.fromkeys(sorted(node_id(p) for p in pubs)))
        sub_ids = list(dict.fromkeys(sorted(node_id(s) for s in subs)))
        if len(pub_ids) == 1 and len(sub_ids) == 1:
            if pub_ids[0] != sub_ids[0]:
                folded[(pub_ids[0], sub_ids[0])].append(topic)
            continue
        tid = mid("T", topic)
        pills[home(pubs)].append(f'{tid}(["{safe(topic)}"]):::topic')
        edges += [f"{pub} --> {tid}" for pub in pub_ids]
        edges += [f"{tid} --> {sub}" for sub in sub_ids]
    for (src, dst), names in sorted(folded.items()):
        edges.append(f'{src} -->|"{topic_list_label(names)}"| {dst}')
    return pills, edges


def render_full(model):
    """Every visible node, grouped by container; containers coloured by Pi."""
    grouped = model.containers()

    def home(pubs):
        owners = {model.container_of(p) for p in pubs}
        return owners.pop() if len(owners) == 1 else "outside"

    pills, edges = topic_flow(model.topics, lambda n: mid("N", n), home)
    lines = ["%% Auto-generated by tools/architecture_runtime_graph.py — full node graph", *HEADER, "flowchart LR"]
    styles = []
    order = sorted((c for c in grouped if c), key=lambda c: (c[0], c[1]))
    if None in grouped:
        order.append(None)
    for container in order:
        cid = container_id(container)
        title = f"🐳 {container[1]} · {container[0]}" if container else "❓ container not proven"
        lines.append(f'    subgraph {cid}["{safe(title)}"]')
        for node in sorted(grouped[container]):
            lines.append(f'        {mid("N", node)}["{safe(node)}"]:::rosnode')
        lines += ["        " + x for x in pills.get(container, [])]
        lines.append("    end")
        if container:
            styles.append(f"    style {cid} {HOST_SUBGRAPH_STYLES[host_class(model, container[0])]}")
        else:
            styles.append(f"    style {cid} fill:#fffbeb,stroke:#d97706,stroke-dasharray:4 3")
    lines += ["    " + x for x in pills.get("outside", [])]
    lines += ["    " + x for x in edges]
    lines += ["", *CLASS_DEFS, *styles]
    return "\n".join(lines) + "\n"


def render_container(model, container):
    """One container's nodes in the middle, peers collapsed to their containers."""
    own = set(model.containers()[container])
    cid = container_id(container)

    def ident(node):
        if node in own:
            return mid("N", node)
        return mid("X", "_".join(model.container_of(node) or UNPLACED))

    externals = {}
    relevant = {}
    for topic, (pubs, subs) in model.topics.items():
        if not (own & (pubs | subs)):
            continue
        # Keep only the half of the traffic that touches this container.
        subs_kept = subs if own & pubs else subs & own
        pubs_kept = pubs if own & subs else pubs & own
        relevant[topic] = (pubs_kept, subs_kept)
        for node in (pubs_kept | subs_kept) - own:
            externals[ident(node)] = model.container_of(node) or UNPLACED

    pills, edges = topic_flow(relevant, ident, lambda pubs: "inside" if pubs <= own else "outside")
    title = f"🐳 {container[1]}" if container else "❓ unplaced nodes"
    lines = [*HEADER, "flowchart LR", f'    subgraph {cid}["{safe(title)}"]']
    lines += [f'        {mid("N", node)}["{safe(node)}"]:::rosnode' for node in sorted(own)]
    lines += ["        " + x for x in pills.get("inside", [])]
    lines.append("    end")
    for xid, (host, name) in sorted(externals.items()):
        where = host if name != UNPLACED[1] else "unplaced"
        lines.append(f'    {xid}["{safe(name)}<br/><small>{safe(where)}</small>"]:::external')
    lines += ["    " + x for x in pills.get("outside", [])]
    lines += ["    " + x for x in edges]
    lines += ["", *CLASS_DEFS]
    if container:
        lines.append(f"    style {cid} {HOST_SUBGRAPH_STYLES[host_class(model, container[0])]}")
    return "\n".join(lines) + "\n"


def render_markdown(model):
    grouped = model.containers()
    placed = sum(1 for n in model.nodes if n in model.placement)
    out = [
        "## ROS2 runtime architecture",
        "",
        f"- ROS nodes shown: **{len(model.nodes)}** (placed in a container: **{placed}**, "
        f"unplaced: **{len(model.nodes) - placed}**)",
        f"- Connected topics drawn: **{len(model.topics)}**; hidden infrastructure topics: "
        f"**{len(model.infra_topics)}**; topics without a peer: **{len(model.dangling)}**",
        f"- Hidden library helper nodes (tf2 listeners, launch process): **{len(model.helper_nodes)}**",
        "- Reading guide: 🖥️ = Raspberry Pi, 🐳 = Docker container, arrows = ROS topics "
        "(publisher → subscriber). A topic with exactly one publisher and one subscriber is "
        "drawn as an arrow label, a shared topic as a green pill.",
        "",
        "### Container map",
        "",
        "```mermaid",
        render_overview(model).rstrip(),
        "```",
        "",
        "### Containers in detail",
        "",
    ]
    order = sorted(
        (c for c in grouped if c), key=lambda c: (list(model.hosts).index(c[0]) if c[0] in model.hosts else 99, c[1])
    )
    if None in grouped:
        order.append(None)
    for container in order:
        title = f"🐳 {container[1]} — {container[0]}" if container else "❓ Unplaced nodes"
        out += [
            f"<details><summary><b>{safe(title)}</b> ({len(grouped[container])} nodes)</summary>",
            "",
            "```mermaid",
            render_container(model, container).rstrip(),
            "```",
            "",
            "</details>",
            "",
        ]

    out += [
        "### Node placement evidence",
        "",
        "| Node | Container | Class | Method | Evidence |",
        "|---|---|---|---|---|",
    ]
    for node in model.nodes:
        meta = model.placement.get(node)
        cls = resolve_class(node, model.classes) or "—"
        if meta:
            where = f"{service_label(meta['service'])} ({meta['service']['host']})"
            row = [node, where, cls, meta["method"], meta["evidence"]]
        else:
            row = [node, "**unplaced**", cls, "—", "no start script / launch file names this node"]
        out.append("| " + " | ".join(safe(x).replace("\n", " ") for x in row) + " |")
    out += [
        "",
        "<details><summary>Hidden infrastructure topics</summary>",
        "",
        "| Topic | Publishers | Subscribers |",
        "|---|---|---|",
    ]
    for topic, (pubs, subs) in sorted(model.infra_topics.items()):
        out.append(f"| {safe(topic)} | {pubs} | {subs} |")
    out += ["", "</details>", ""]
    if model.helper_nodes:
        out += [
            "<details><summary>Hidden helper nodes</summary>",
            "",
            *[f"- `{n}`" for n in model.helper_nodes],
            "",
            "</details>",
            "",
        ]
    if model.dangling:
        out += [
            "<details><summary>Topics with publishers or subscribers only</summary>",
            "",
            *[f"- `{t}`" for t in model.dangling],
            "",
            "</details>",
            "",
        ]
    out.append("Full node-level graph: `architecture/runtime-graph.mmd` in the `architecture-local-runtime` artifact.")
    return "\n".join(out) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime", type=Path, default=Path("architecture/runtime.json"))
    parser.add_argument("--inventory", type=Path, default=Path("architecture/inventory.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("architecture"))
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    args = parser.parse_args()

    runtime = json.loads(args.runtime.read_text(encoding="utf-8"))
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=True)

    model = Model(runtime, inventory, args.repo_root)
    (args.output_dir / "runtime-overview.mmd").write_text(render_overview(model), encoding="utf-8")
    (args.output_dir / "runtime-graph.mmd").write_text(render_full(model), encoding="utf-8")
    (args.output_dir / "runtime-graph.md").write_text(render_markdown(model), encoding="utf-8")

    unplaced = [n for n in model.nodes if n not in model.placement]
    print(
        json.dumps(
            {
                "runtime_nodes": len(runtime.get("nodes", [])),
                "shown_nodes": len(model.nodes),
                "hidden_helper_nodes": len(model.helper_nodes),
                "connected_topics": len(model.topics),
                "hidden_infra_topics": len(model.infra_topics),
                "containers": len([c for c in model.containers() if c]),
                "container_edges": len(model.container_edges()),
                "unresolved_class_nodes": sum(resolve_class(n, model.classes) is None for n in model.nodes),
                "unplaced_nodes": unplaced,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
