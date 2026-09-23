"""
统一异常体系 (errors.py)

作者: 模具自动生成系统开发组
用途: 定义模具生成系统的自定义异常。所有模块（读取、对齐、布尔运算、分模、导出等）
      遇到无法自动恢复的错误时，统一抛出 MoldGenerationError 或其子类，
      便于上层集中捕获、记录日志并给出清晰错误提示。
"""


class MoldGenerationError(Exception):
    """模具生成系统的基础异常。

    所有模块抛出的异常都应继承本类。携带 stage 属性用于标识出错阶段。
    """

    def __init__(self, message: str, *, stage: str = "unknown") -> None:
        super().__init__(message)
        self.stage = stage          # 出错阶段标识（如 model_reader / cavity_cutter ...）
        self.message = message

    def __str__(self) -> str:
        return f"[{self.stage}] {self.message}"


class ModelReadError(MoldGenerationError):
    """产品数模读取失败（文件缺失、格式不支持、解析失败等）。"""

    def __init__(self, message: str) -> None:
        super().__init__(message, stage="model_reader")


class GeometryError(MoldGenerationError):
    """几何计算/校验失败（包围盒、体积、有效性等）。"""

    def __init__(self, message: str) -> None:
        super().__init__(message, stage="geometry")


class BooleanOpError(MoldGenerationError):
    """布尔运算失败或结果无效。"""

    def __init__(self, message: str) -> None:
        super().__init__(message, stage="boolean_op")


class PartingError(MoldGenerationError):
    """分模失败（分模高度非法、上下模体积异常等）。"""

    def __init__(self, message: str) -> None:
        super().__init__(message, stage="parting_splitter")


class ExportError(MoldGenerationError):
    """结果导出失败。"""

    def __init__(self, message: str) -> None:
        super().__init__(message, stage="result_exporter")


if __name__ == "__main__":
    # 自测入口
    for exc in (ModelReadError("示例"), GeometryError("示例"), BooleanOpError("示例"),
                PartingError("示例"), ExportError("示例")):
        print(exc)
