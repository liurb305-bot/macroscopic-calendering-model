param(
    [string]$WorkDir = 'E:\abaqus\3Dfuxian'
)

Add-Type -AssemblyName System.Drawing

$inputCsv = Join-Path $WorkDir 'D900_10pct_vs_20pct_CPRESS_profile_X.csv'
$outputPng = Join-Path $WorkDir 'D900_10pct_vs_20pct_CPRESS_profile_X.png'
$data = Import-Csv -LiteralPath $inputCsv

$width = 1200
$height = 780
$left = 105
$right = 35
$top = 65
$bottom = 90
$plotWidth = $width - $left - $right
$plotHeight = $height - $top - $bottom
$xMin = 0.0
$xMax = 10.0
$allPressure = @($data.CPRESS_10pct_width_and_surface_average_MPa) +
    @($data.CPRESS_20pct_width_and_surface_average_MPa)
$pressureMax = ($allPressure | ForEach-Object { [double]$_ } |
    Measure-Object -Maximum).Maximum
$yMin = 0.0
$yMax = [Math]::Ceiling(($pressureMax + 10.0) / 20.0) * 20.0

function To-X([double]$value) {
    return $left + (($value - $xMin) / ($xMax - $xMin)) * $plotWidth
}

function To-Y([double]$value) {
    return $top + (($yMax - $value) / ($yMax - $yMin)) * $plotHeight
}

$bitmap = New-Object System.Drawing.Bitmap($width, $height)
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
$graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$graphics.Clear([System.Drawing.Color]::White)

$font = New-Object System.Drawing.Font('Arial', 13)
$smallFont = New-Object System.Drawing.Font('Arial', 11)
$titleFont = New-Object System.Drawing.Font('Arial', 20, [System.Drawing.FontStyle]::Bold)
$axisPen = New-Object System.Drawing.Pen([System.Drawing.Color]::Black, 2)
$gridPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(225, 225, 225), 1)
$bluePen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(35, 75, 130), 3)
$redPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(220, 45, 45), 3)
$centerPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(100, 100, 100), 1.5)
$centerPen.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
$blackBrush = [System.Drawing.Brushes]::Black
$blueBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(35, 75, 130))
$redBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(220, 45, 45))

for ($x = 0; $x -le 10; $x += 1) {
    $px = [single](To-X $x)
    $graphics.DrawLine($gridPen, $px, $top, $px, $top + $plotHeight)
    $label = $x.ToString('0')
    $size = $graphics.MeasureString($label, $smallFont)
    $graphics.DrawString($label, $smallFont, $blackBrush,
        $px - $size.Width / 2, $top + $plotHeight + 10)
}

for ($y = 0; $y -le $yMax + 1.0e-8; $y += 20) {
    $py = [single](To-Y $y)
    $graphics.DrawLine($gridPen, $left, $py, $left + $plotWidth, $py)
    $label = $y.ToString('0')
    $size = $graphics.MeasureString($label, $smallFont)
    $graphics.DrawString($label, $smallFont, $blackBrush,
        $left - $size.Width - 10, $py - $size.Height / 2)
}

$graphics.DrawRectangle($axisPen, $left, $top, $plotWidth, $plotHeight)
$graphics.DrawLine($centerPen, (To-X 5.0), $top,
    (To-X 5.0), $top + $plotHeight)

for ($i = 1; $i -lt $data.Count; $i++) {
    $a = $data[$i - 1]
    $b = $data[$i]
    $graphics.DrawLine($bluePen,
        (To-X ([double]$a.film_length_coordinate_mm)),
        (To-Y ([double]$a.CPRESS_10pct_width_and_surface_average_MPa)),
        (To-X ([double]$b.film_length_coordinate_mm)),
        (To-Y ([double]$b.CPRESS_10pct_width_and_surface_average_MPa)))
    $graphics.DrawLine($redPen,
        (To-X ([double]$a.film_length_coordinate_mm)),
        (To-Y ([double]$a.CPRESS_20pct_width_and_surface_average_MPa)),
        (To-X ([double]$b.film_length_coordinate_mm)),
        (To-Y ([double]$b.CPRESS_20pct_width_and_surface_average_MPa)))
}

for ($i = 0; $i -lt $data.Count; $i += 5) {
    $row = $data[$i]
    $x = [single](To-X ([double]$row.film_length_coordinate_mm))
    $y10 = [single](To-Y ([double]$row.CPRESS_10pct_width_and_surface_average_MPa))
    $y20 = [single](To-Y ([double]$row.CPRESS_20pct_width_and_surface_average_MPa))
    $graphics.FillRectangle($blueBrush, $x - 3, $y10 - 3, 6, 6)
    $graphics.FillEllipse($redBrush, $x - 3, $y20 - 3, 6, 6)
}

$title = 'D900 contact pressure along film length: 10% vs 20%'
$titleSize = $graphics.MeasureString($title, $titleFont)
$graphics.DrawString($title, $titleFont, $blackBrush,
    ($width - $titleSize.Width) / 2, 15)
$xTitle = 'Film length coordinate (mm)'
$xTitleSize = $graphics.MeasureString($xTitle, $font)
$graphics.DrawString($xTitle, $font, $blackBrush,
    ($width - $xTitleSize.Width) / 2, $height - 43)
$graphics.TranslateTransform(28, $height / 2)
$graphics.RotateTransform(-90)
$yTitle = 'Width-averaged contact pressure CPRESS (MPa)'
$yTitleSize = $graphics.MeasureString($yTitle, $font)
$graphics.DrawString($yTitle, $font, $blackBrush, -$yTitleSize.Width / 2, 0)
$graphics.ResetTransform()

$legendX = $left + 25
$legendY = $top + 25
$graphics.DrawLine($bluePen, $legendX, $legendY, $legendX + 42, $legendY)
$graphics.DrawString('10% reduction, Hold', $font, $blackBrush,
    $legendX + 52, $legendY - 10)
$graphics.DrawLine($redPen, $legendX, $legendY + 32, $legendX + 42, $legendY + 32)
$graphics.DrawString('20% reduction, Hold', $font, $blackBrush,
    $legendX + 52, $legendY + 22)
$graphics.DrawLine($centerPen, $legendX, $legendY + 64, $legendX + 42, $legendY + 64)
$graphics.DrawString('Deformation-zone center', $font, $blackBrush,
    $legendX + 52, $legendY + 54)

$bitmap.Save($outputPng, [System.Drawing.Imaging.ImageFormat]::Png)
$graphics.Dispose()
$bitmap.Dispose()
$font.Dispose()
$smallFont.Dispose()
$titleFont.Dispose()
$axisPen.Dispose()
$gridPen.Dispose()
$bluePen.Dispose()
$redPen.Dispose()
$centerPen.Dispose()
$blueBrush.Dispose()
$redBrush.Dispose()

Write-Output $outputPng
