"""rob_box_supervisor — Avatar Supervisor (single coordinator for Quest/Telegram clients).

Skeleton package (AV-2) per `docs/adr/0028-avatar-supervisor.md` §4.6.
The actual FSM, LockManager, dispatcher and aggregator land in later cards
(AV-3..AV-6). For Phase 1 (AV-9) the supervisor runs in monitor-only mode
and just publishes ``/avatar/state``.

Public surface kept intentionally small for the skeleton phase:
    - ``__version__`` (semver string, sync'd with ``package.xml``/``setup.py``)
    - the ``rob_box_supervisor.supervisor_node`` console-script module
      (its ``main()`` is the entry point declared in ``setup.py``).
"""

__version__ = "0.1.0"
