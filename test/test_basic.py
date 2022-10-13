import pathlib
from system_fingerprint.imprint import main


def test_basic_smoke():
    """Run main, hope for no errors"""
    main([])

    output_file = pathlib.Path('fingerprint.yaml')
    if output_file.exists():
        output_file.unlink()


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('system_fingerprint', 'test_basic_smoke')
