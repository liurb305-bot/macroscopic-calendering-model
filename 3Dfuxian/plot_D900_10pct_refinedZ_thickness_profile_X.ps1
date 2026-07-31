param(
    [string]$WorkDir = 'E:\abaqus\3Dfuxian'
)

Add-Type -AssemblyName System.Drawing

$inputCsv = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D900_10pct_RefinedZ_thickness_profile_X.csv'
$outputPng = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D900_10pct_RefinedZ_thickness_profile_X.png'
$data = Import-Csv -LiteralPath $inputCsv

$width = 1200
$height = 780
$left = 105
$right = 35
$top = 65
$bottom = 90
$plotWidth = $width - $left - $right
$plotHeight = $height - $top - $bottom
$xMin = -5.0
$xMax = 5.0
$values = $data | ForEach-Object { [double]$_.D900_refinedZ_10pct_hold_thickness_um }
$minimum = ($values | Measure-Object -Minimum).Minimum
$maximum = ($values | Measure-Object -Maximum).Maximum
$yMin = [Math]::Floor(($minimum - 0.10) * 10.0) / 10.0
$yMax = [Math]::Ceiling(($maximum + 0.10) * 10.0) / 10.0

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
$redPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(220, 45, 45), 3)
$grayPen = New-Object System.Drawing.Pen([System.Drawing.Color]::Gray, 2)
$grayPen.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
$contactPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(60, 110, 170), 1.5)
$contactPen.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dot
$blackBrush = [System.Drawing.Brushes]::Black
$redBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(220, 45, 45))

for ($x = -5; $x -le 5; $x += 1) {
    $px = [single](To-X $x)
    $graphics.DrawLine($gridPen, $px, $top, $px, $top + $plotHeight)
    $label = $x.ToString('0')
    $size = $graphics.MeasureString($label, $smallFont)
    $graphics.DrawString($label, $smallFont, $blackBrush,
        $px - $size.Width / 2, $top + $plotHeight + 10)
}

$tickStart = [Math]::Ceiling($yMin * 2.0) / 2.0
for ($y = $tickStart; $y -le $yMax + 1.0e-8; $y += 0.5) {
    $py = [single](To-Y $y)
    $graphics.DrawLine($gridPen, $left, $py, $left + $plotWidth, $py)
    $label = $y.ToString('0.0')
    $size = $graphics.MeasureString($label, $smallFont)
    $graphics.DrawString($label, $smallFont, $blackBrush,
        $left - $size.Width - 10, $py - $size.Height / 2)
}

$graphics.DrawRectangle($axisPen, $left, $top, $plotWidth, $plotHeight)
$graphics.DrawLine($grayPen, (To-X $xMin), (To-Y 150.0), (To-X $xMax), (To-Y 150.0))
foreach ($contactX in @(-2.6, 2.6)) {
    $graphics.DrawLine($contactPen, (To-X $contactX), $top,
        (To-X $contactX), $top + $plotHeight)
}

for ($i = 1; $i -lt $data.Count; $i++) {
    $a = $data[$i - 1]
    $b = $data[$i]
    $graphics.DrawLine($redPen,
        (To-X ([double]$a.X_distance_from_deformation_center_mm)),
        (To-Y ([double]$a.D900_refinedZ_10pct_hold_thickness_um)),
        (To-X ([double]$b.X_distance_from_deformation_center_mm)),
        (To-Y ([double]$b.D900_refinedZ_10pct_hold_thickness_um)))
}

for ($i = 0; $i -lt $data.Count; $i += 5) {
    $row = $data[$i]
    $x = [single](To-X ([double]$row.X_distance_from_deformation_center_mm))
    $y = [single](To-Y ([double]$row.D900_refinedZ_10pct_hold_thickness_um))
    $graphics.FillEllipse($redBrush, $x - 3, $y - 3, 6, 6)
}

$title = 'D900, 10% reduction: thickness along rolling direction X'
$titleSize = $graphics.MeasureString($title, $titleFont)
$graphics.DrawString($title, $titleFont, $blackBrush,
    ($width - $titleSize.Width) / 2, 15)
$xTitle = 'Distance from deformation-zone center X (mm)'
$xTitleSize = $graphics.MeasureString($xTitle, $font)
$graphics.DrawString($xTitle, $font, $blackBrush,
    ($width - $xTitleSize.Width) / 2, $height - 43)
$graphics.TranslateTransform(28, $height / 2)
$graphics.RotateTransform(-90)
$yTitle = 'Film thickness (um)'
$yTitleSize = $graphics.MeasureString($yTitle, $font)
$graphics.DrawString($yTitle, $font, $blackBrush, -$yTitleSize.Width / 2, 0)
$graphics.ResetTransform()

$legendX = $left + 25
$legendY = $top + 25
$graphics.DrawLine($redPen, $legendX, $legendY, $legendX + 42, $legendY)
$graphics.DrawString('Hold thickness at Z=0', $font, $blackBrush,
    $legendX + 52, $legendY - 10)
$graphics.DrawLine($grayPen, $legendX, $legendY + 32, $legendX + 42, $legendY + 32)
$graphics.DrawString('Initial thickness 150 um', $font, $blackBrush,
    $legendX + 52, $legendY + 22)
$graphics.DrawLine($contactPen, $legendX, $legendY + 64, $legendX + 42, $legendY + 64)
$graphics.DrawString('Nominal contact bounds X=+/-2.6 mm', $font, $blackBrush,
    $legendX + 52, $legendY + 54)

$bitmap.Save($outputPng, [System.Drawing.Imaging.ImageFormat]::Png)
$graphics.Dispose()
$bitmap.Dispose()
$font.Dispose()
$smallFont.Dispose()
$titleFont.Dispose()
$axisPen.Dispose()
$gridPen.Dispose()
$redPen.Dispose()
$grayPen.Dispose()
$contactPen.Dispose()
$redBrush.Dispose()

Write-Output $outputPng
