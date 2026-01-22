try:
    from recording import data_record
except ModuleNotFoundError:
    import sys
    from pathlib import Path

    ROOT_DIR = Path(__file__).resolve().parent.parent
    root_str = str(ROOT_DIR)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    from recording import data_record


def build_ch3_recorder(parent_window, data_dir, imageprocess, get_attacker, get_cmd, log_func):
    return data_record.Ch3Recorder(
        parent_window=parent_window,
        data_dir=data_dir,
        imageprocess=imageprocess,
        get_attacker=get_attacker,
        get_cmd=get_cmd,
        log_func=log_func,
    )
