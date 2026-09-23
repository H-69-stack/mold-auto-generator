"""
文件 I/O 工具 (io_utils.py)

作者: 模具自动生成系统开发组
用途: 处理 OpenCASCADE 底层文件 API 对非 ASCII 路径（如中文用户名目录）支持不佳的问题：
      - 读取前：若路径含非 ASCII 字符，先复制到 ASCII 临时路径再交给 OCC 读取；
      - 写入前：先让 OCC 写到 ASCII 临时路径，再用 Python 移动回目标路径
        （Python 使用 Windows Unicode API，可正确处理中文路径）。
"""

import os
import shutil
import sys
import tempfile
import uuid
from pathlib import Path
from typing import Tuple


def ensure_utf8_stdout() -> None:
    """保证 Windows 控制台 / 管道下中文与单位符号（mm³、mm² 等）输出不报编码错误。"""
    for stream in (sys.stdout, sys.stderr):
        if stream is not None and hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:  # noqa: BLE001
                pass


def is_ascii_path(path: str) -> bool:
    """判断路径是否全部为 ASCII 字符。"""
    try:
        path.encode("ascii")
        return True
    except UnicodeEncodeError:
        return False


def ascii_temp_root() -> Path:
    """返回一个真正可写的 ASCII 临时目录。

    不信任 os.access 的表层结果（沙箱下常出现“名义可写、实际写入被拒”），
    改用真实写入探测：候选目录逐一尝试写一个探针文件，首个写成功的返回。
    均不可用时退回系统临时目录（可能含非 ASCII，属最后手段）。
    """
    candidates = [
        Path(os.environ.get("SystemDrive", "C:") + r"\occ_tmp"),
        Path("C:/occ_tmp"),
        Path("D:/occ_tmp"),
        Path("E:/occ_tmp"),
        Path("C:/Users/Public/occ_tmp"),
        Path(tempfile.gettempdir()),
    ]
    for cand in candidates:
        try:
            cand.mkdir(parents=True, exist_ok=True)
            probe = cand / f".probe_{uuid.uuid4().hex[:6]}"
            probe.write_text("x", encoding="ascii")
            probe.unlink()
            return cand
        except OSError:
            continue
    return Path(".")


def make_ascii_read_path(file_path: str) -> Tuple[str, bool]:
    """若路径含非 ASCII 字符，复制到 ASCII 临时路径供 OCC 读取。

    返回 (实际用于 OCC 读取的路径, 是否为临时副本)。
    """
    src = Path(file_path)
    if not src.exists():
        raise FileNotFoundError(f"文件不存在: {file_path}")
    if is_ascii_path(str(src)):
        return str(src), False
    tmp = ascii_temp_root() / f"{src.stem}_{uuid.uuid4().hex[:8]}{src.suffix}"
    shutil.copy2(src, tmp)
    return str(tmp), True


def make_ascii_write_path(file_path: str) -> Tuple[str, bool]:
    """若目标路径含非 ASCII 字符，返回 ASCII 临时写入路径。

    返回 (实际写入路径, 是否需要在写完后移动回原路径)。
    """
    dst = Path(file_path)
    if is_ascii_path(str(dst)):
        return str(dst), False
    tmp = ascii_temp_root() / f"{dst.stem}_{uuid.uuid4().hex[:8]}{dst.suffix}"
    return str(tmp), True


def move_into_place(tmp_path: str, final_path: str) -> None:
    """把临时文件移动到最终路径（由 Python 完成，支持中文路径）。"""
    tmp = Path(tmp_path)
    final = Path(final_path)
    if tmp.resolve() == final.resolve():
        return
    final.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(tmp), str(final))


if __name__ == "__main__":
    # 自测入口
    print("ASCII 临时目录:", ascii_temp_root())
    demo = r"C:\Users\测试用户\demo.stp"
    tmp, is_tmp = make_ascii_write_path(demo)
    print(f"写入中转: {demo!r} -> {tmp!r} (is_tmp={is_tmp})")
    print("ASCII 路径检测 'C:/a.stp':", is_ascii_path("C:/a.stp"))
