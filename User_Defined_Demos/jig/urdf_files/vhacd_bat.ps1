# 设置 VHACD 程序的路径
$vhacdPath = "D:\Download\Edge download\v-hacd-4.1.0\app\TestVHACD.exe"
# 设置要处理的文件夹路径
$directoryPath = "F:\sw\urdf_files\Link_C_18parts\processed-geoms"
# 设置输出文件夹路径
$outputDirectory = "F:\sw\urdf_files\Link_C_18parts\processed-geoms\output"

# 创建输出文件夹（如果不存在）
if (-not (Test-Path $outputDirectory)) {
    New-Item -ItemType Directory -Path $outputDirectory
}

# 遍历文件夹中的所有 .obj 文件
Get-ChildItem -Path $directoryPath -Filter *.obj | ForEach-Object {
    $inputFile = $_.FullName
    $outputBaseName = $_.BaseName + "_decomp"
    $outputObjFile = Join-Path $outputDirectory ($outputBaseName + ".obj")

    # 构建并执行 VHACD 命令
    & $vhacdPath $inputFile -h 64 -r 1000000 -e 0.01 -d 16

    # 检查 decomp.obj 文件是否存在，并移动重命名
    $decompPath = "decomp.obj"
    if (Test-Path $decompPath) {
        Move-Item $decompPath $outputObjFile -Force
    }
    Write-Output "Processed $inputFile, saved results to $outputObjFile"
}

# Get-ExecutionPolicy
# Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
