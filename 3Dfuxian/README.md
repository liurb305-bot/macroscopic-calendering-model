# 3Dfuxian

本目录用于存放自支撑膜三维局部静态辊压模型相关文件。

## 内容

- Abaqus/CAE 建模脚本与 ODB 后处理脚本。
- 已导出的 `.cae` 与 `.inp` 模型文件。
- 厚度、接触压力、反力历史等后处理 CSV 文件。
- 对比图 PNG、Excel/Origin 绘图数据文件。

## 说明

- 模型为 Abaqus/Standard 静态压下模型，不包含完整动态辊压、辊子转动或极片进料。
- 极片为单层自支撑膜，相关材料参数按项目脚本中的集中参数区定义。
- 仓库 `.gitignore` 会忽略 `.odb/.jnl/.msg/.sta/.dat/.rec` 等大型求解结果与临时文件，因此 ODB 结果文件没有上传到 GitHub。
