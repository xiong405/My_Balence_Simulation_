# URDF 导入 Webots：精简操作版

本文只保留最核心的操作：

1. 创建 Webots 工程目录；
2. 创建 Webots 专用 URDF；
3. 把 URDF 转换成 PROTO；
4. 把 Windows 路径中的 `\` 改成 `/`；
5. 在 Webots 中添加 PROTO。

示例工程路径：

```text
C:\Users\MECHREVO\Desktop\My_simscape\mY_webots\test3
```

## 一、准备目录

打开 PowerShell，执行：

```powershell
$p = "C:\Users\MECHREVO\Desktop\My_simscape\mY_webots\test3"

New-Item -ItemType Directory -Force "$p\protos" | Out-Null
New-Item -ItemType Directory -Force "$p\worlds" | Out-Null
New-Item -ItemType Directory -Force "$p\controllers" | Out-Null
```

完成后的核心目录为：

```text
test3/
├─ meshes/          # SolidWorks 导出的 STL 外形
├─ urdf/            # SolidWorks 导出的 URDF
├─ protos/          # 转换后生成的 Webots PROTO
├─ worlds/          # Webots 世界文件 WBT
└─ controllers/     # Webots 控制器
```

## 二、创建 Webots 专用 URDF

原始 URDF 中的网格路径通常是：

```text
package://test model-assembly/meshes/xxx.STL
```

这种路径需要 ROS。当前只使用 Webots，所以要把它改为：

```text
../meshes/xxx.STL
```

执行：

```powershell
$p = "C:\Users\MECHREVO\Desktop\My_simscape\mY_webots\test3"
$sourceUrdf = "$p\urdf\test model-assembly.urdf"
$fixedUrdf  = "$p\urdf\test_model_assembly_webots.urdf"

$text = [System.IO.File]::ReadAllText($sourceUrdf)
$text = [System.Text.RegularExpressions.Regex]::Replace(
    $text,
    'package://[^/]+/meshes/',
    '../meshes/'
)

[System.IO.File]::WriteAllText(
    $fixedUrdf,
    $text,
    [System.Text.UTF8Encoding]::new($false)
)
```

检查新文件是否生成：

```powershell
Test-Path $fixedUrdf
```

应返回：

```text
True
```

检查是否还存在 ROS 路径：

```powershell
Select-String -LiteralPath $fixedUrdf -Pattern "package://"
```

正确情况下没有任何输出。

## 三、将 URDF 转换成 PROTO

执行：

```powershell
$python = "C:\Users\MECHREVO\AppData\Local\Programs\Python\Python312\python.exe"
$out = "$p\protos\TestModelAssembly.proto"

& $python -X utf8 -m urdf2webots.importer --input="$fixedUrdf" --output="$out" --normal
```

其中：

- `-X utf8`：避免 Windows GBK 编码错误；
- `--input`：指定输入 URDF；
- `--output`：指定输出 PROTO；
- `--normal`：保留网格法向量，使模型表面显示正常。

成功时会显示类似：

```text
Root link: base_link
There are 13 links, 12 joints and 0 sensors
```

检查 PROTO：

```powershell
Test-Path $out
```

应返回：

```text
True
```

## 四、把 Windows 反斜杠改成正斜杠

Windows 转换器有时会在 PROTO 中生成：

```webots
url "..\meshes\base_link.STL"
```

Webots 资源路径应使用 `/`：

```webots
url "../meshes/base_link.STL"
```

执行：

```powershell
$protoText = [System.IO.File]::ReadAllText($out)
$protoText = $protoText.Replace('\', '/')

[System.IO.File]::WriteAllText(
    $out,
    $protoText,
    [System.Text.UTF8Encoding]::new($false)
)
```

这一步只是把已有路径中的 `\` 替换为 `/`，不会新增第二个 `url` 字段。

检查所有 URL：

```powershell
Select-String -LiteralPath $out -Pattern "url"
```

应看到类似：

```text
url "../meshes/base_link.STL"
url "../meshes/r1.STL"
url "../meshes/wheel.STL"
```

每个 `Mesh {}` 中只能有一个 `url`，不要手动再添加第二个。

## 五、在 Webots 中导入

1. 打开 Webots；
2. 新建世界；
3. 将世界保存到：

   ```text
   test3\worlds\test3.wbt
   ```

4. 在 Scene Tree 点击 `+`；
5. 选择：

   ```text
   PROTO nodes (Current Project)
   ```

6. 选择：

   ```text
   TestModelAssembly
   ```

7. 点击 Add；
8. 将机器人 `controller` 设置为：

   ```text
   <none>
   ```

9. 设置初始位置，例如：

   ```text
   translation 0 0 0.5
   ```

10. 点击 Reset，确认模型正常显示。

## 六、完整命令汇总

下面这段可以直接复制到 PowerShell：

```powershell
$p = "C:\Users\MECHREVO\Desktop\My_simscape\mY_webots\test3"
$python = "C:\Users\MECHREVO\AppData\Local\Programs\Python\Python312\python.exe"
$sourceUrdf = "$p\urdf\test model-assembly.urdf"
$fixedUrdf = "$p\urdf\test_model_assembly_webots.urdf"
$out = "$p\protos\TestModelAssembly.proto"

New-Item -ItemType Directory -Force "$p\protos" | Out-Null
New-Item -ItemType Directory -Force "$p\worlds" | Out-Null
New-Item -ItemType Directory -Force "$p\controllers" | Out-Null

$text = [System.IO.File]::ReadAllText($sourceUrdf)
$text = [System.Text.RegularExpressions.Regex]::Replace(
    $text,
    'package://[^/]+/meshes/',
    '../meshes/'
)
[System.IO.File]::WriteAllText(
    $fixedUrdf,
    $text,
    [System.Text.UTF8Encoding]::new($false)
)

& $python -X utf8 -m urdf2webots.importer --input="$fixedUrdf" --output="$out" --normal

$protoText = [System.IO.File]::ReadAllText($out)
$protoText = $protoText.Replace('\', '/')
[System.IO.File]::WriteAllText(
    $out,
    $protoText,
    [System.Text.UTF8Encoding]::new($false)
)

Write-Host "PROTO 已生成：$out" -ForegroundColor Green
Select-String -LiteralPath $out -Pattern "url"
```

最终只需要记住：

```text
原始 URDF
→ 替换 package:// 路径
→ urdf2webots 转换
→ 把 \ 改成 /
→ 在 Webots 中添加 PROTO
```
