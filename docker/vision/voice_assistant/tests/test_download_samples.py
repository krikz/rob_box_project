import importlib.util
import pathlib


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "download_samples.py"


def load_module():
    spec = importlib.util.spec_from_file_location("download_samples", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_iter_collection_files_preserves_absolute_urls() -> None:
    module = load_module()

    tree = {
        "name": "1_pitchglitch_samples",
        "path": "./samples/1_pitchglitch_samples",
        "children": [
            {
                "name": "g",
                "path": "./samples/1_pitchglitch_samples/g",
                "children": [
                    {
                        "name": "upper",
                        "path": "./samples/1_pitchglitch_samples/g/upper",
                        "children": [
                            {
                                "name": "023_Various_Guitar_PetitBarcelona.wav",
                                "path": "./samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
                                "url": "https://collections.renardo.org/samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
                            }
                        ],
                    }
                ],
            }
        ],
    }

    entries = list(module.iter_collection_files(tree))

    assert entries == [
        (
            "https://collections.renardo.org/samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
            pathlib.Path("1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav"),
        )
    ]


def test_download_collection_retries_failures_sequentially(tmp_path) -> None:
    module = load_module()

    tree = {
        "name": "1_pitchglitch_samples",
        "path": "./samples/1_pitchglitch_samples",
        "children": [
            {
                "name": "g",
                "path": "./samples/1_pitchglitch_samples/g",
                "children": [
                    {
                        "name": "upper",
                        "path": "./samples/1_pitchglitch_samples/g/upper",
                        "children": [
                            {
                                "name": "023_Various_Guitar_PetitBarcelona.wav",
                                "path": "./samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
                                "url": "https://collections.renardo.org/samples/1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav",
                            }
                        ],
                    }
                ],
            }
        ],
    }

    attempts = {"count": 0}

    def fake_load_index(_json_url, _logger):
        return tree

    def fake_download_file(*, url, destination, logger, timeout_seconds, retries):
        attempts["count"] += 1
        if attempts["count"] == 1:
            return False
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(b"wav")
        return True

    module.load_json_index = fake_load_index
    module.download_file = fake_download_file

    failures = module.download_collection(
        json_url="https://collections.renardo.org/samples/1_pitchglitch_samples/collection_index.json",
        download_dir=tmp_path,
        logger=module.Logger(),
        max_workers=4,
    )

    assert failures == []
    assert attempts["count"] == 2
    assert (tmp_path / "1_pitchglitch_samples/g/upper/023_Various_Guitar_PetitBarcelona.wav").read_bytes() == b"wav"
