"""管件编号 / 文件名前缀工具（web 与命令行共用）。

用途：从上传文件名里提取**管件编号**作为输出文件名前缀。
例：``24TK_1324-31_ROTATE_x_CW_90deg_ROTATE_y_CW_4deg.stp`` → ``24TK_1324-31``。

规则：以 ``_`` / 空格 / ``---`` 切分，从第一个 token 开始取，直到遇到
ROTATE / MIRROR / CAVITY / PUNCH 之类的后缀 token 为止；纯标点/序号 token
（如 ``---(1)``）丢掉。
"""

import os
import re

# 这些 token 开始就是"操作后缀"，不属于编号
SUFFIX_WORDS = {
    "rotate", "rot", "mirror", "mir", "cavity", "punch", "filled", "fill",
    "hollow", "hollowed", "closed", "copy", "model", "part", "product",
    "output", "final", "ok", "new", "v",
}


def strip_uid_prefix(filename: str) -> str:
    """去掉上传时加的 12 位 hex 存储前缀（``f3a9c1d0e2b4_原名``）。"""
    base = os.path.basename(str(filename))
    while True:
        parts = base.split("_", 1)
        if (len(parts) == 2 and len(parts[0]) == 12
                and all(c in "0123456789abcdef" for c in parts[0].lower())
                and any(c in "abcdef" for c in parts[0].lower())):
            base = parts[1]
            continue
        break
    return base


def part_number_from_filename(filename: str) -> str:
    """从文件名提取管件编号（用作输出前缀）。取不到时退回原文件名主干。"""
    base = os.path.splitext(strip_uid_prefix(filename))[0]
    base = base.replace("---", "_")
    tokens = [t for t in re.split(r"[_\s]+", base) if t]
    out = []
    for tok in tokens:
        low = tok.lower().strip(".-")
        if not low:
            continue
        if low in SUFFIX_WORDS or "rotate" in low or "mirror" in low:
            break
        if not re.search(r"[0-9A-Za-z]", tok):           # 纯标点 token
            continue
        if re.search(r"[()\[\]{}]", tok):                # "(1)" / "---(1)" 这类序号
            continue
        out.append(tok.strip(".-_"))
    name = "_".join(t for t in out if t)
    return name or base or "part"


if __name__ == "__main__":
    import sys

    tests = sys.argv[1:] or [
        "24TK_1324-31_ROTATE_x_CW_90deg_ROTATE_y_CW_4deg.stp",
        "24TK_2052-2_ROTATE_x_CW_90deg.stp",
        "24TK_1324-2.stp",
        "24TK_1324-1_ROTATE_x_CCW_90deg.stp",
        "f3a9c1d0e2b4_24TK_1442-1.stp",
        "YA-1131-505_MIRROR_xOy.stp",
        "2ROM31-6061 ---(1).stp",
    ]
    for t in tests:
        print(f"{t:60s} -> {part_number_from_filename(t)}")
