import tkinter as tk
from pathlib import Path


def _list_strategy_modules(src_dir):
    """List available strategy modules under src/strategies (exclude dunder)."""
    strat_dir = Path(src_dir) / "strategies"
    items = []
    for path in strat_dir.glob("*.py"):
        stem = path.stem
        if stem.startswith("_"):
            continue
        if stem in ("__init__",):
            continue
        items.append(stem)
    return sorted(items)


def choose_strategy(parent_window, src_dir, title, default_name):
    """Popup to choose a strategy module name."""
    names = _list_strategy_modules(src_dir)
    if not names:
        return default_name
    choice = [default_name if default_name in names else names[0]]

    dlg = tk.Toplevel(parent_window)
    dlg.title(title)
    tk.Label(dlg, text="選擇策略").pack(padx=8, pady=6)
    lb = tk.Listbox(dlg, height=min(10, len(names)))
    for n in names:
        lb.insert(tk.END, n)
    try:
        idx = names.index(choice[0])
        lb.select_set(idx)
        lb.see(idx)
    except ValueError:
        lb.select_set(0)
    lb.pack(padx=8, pady=4, fill="both", expand=True)

    def on_ok():
        sel = lb.curselection()
        if sel:
            choice[0] = names[sel[0]]
        dlg.destroy()

    tk.Button(dlg, text="確定", command=on_ok).pack(pady=6)
    dlg.grab_set()
    dlg.focus_force()
    parent_window.wait_window(dlg)
    return choice[0]
