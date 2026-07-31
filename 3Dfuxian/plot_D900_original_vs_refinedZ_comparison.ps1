param(
    [string]$WorkDir = 'E:\abaqus\3Dfuxian'
)

Add-Type -AssemblyName System.Drawing

$oldCsv = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_thickness_profile_Z.csv'
$newCsv = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D900_10pct_RefinedZ_thickness_profile_Z.csv'
$outCsv = Join-Path $WorkDir 'D900_original_vs_refinedZ_thickness_profile_Z.csv'
$outPng = Join-Path $WorkDir 'D900_original_vs_refinedZ_thickness_profile_Z.png'

$oldData = Import-Csv -LiteralPath $oldCsv
$newData = Import-Csv -LiteralPath $newCsv
$newMap = @{}
foreach ($row in $newData) {
    $newMap[[double]$row.Z_distance_from_center_mm] =
        [double]$row.D900_refinedZ_no_unload_10pct_hold_thickness_um
}

$comparison = foreach ($row in $oldData) {
    $z = [double]$row.Z_distance_from_center_mm
    $oldValue = [double]$row.no_unload_hold_thickness_um
    $newValue = $newMap[$z]
    [pscustomobject]@{
        Z_distance_from_center_mm = $z
        D900_original_thickness_um = $oldValue
        D900_refinedZ_thickness_um = $newValue
        refined_minus_original_um = $newValue - $oldValue
    }
}
$comparison | Export-Csv -LiteralPath $outCsv -NoTypeInformation -Encoding UTF8

$width = 1200
$height = 780
$left = 105
$right = 35
$top = 65
$bottom = 90
$plotWidth = $width - $left - $right
$plotHeight = $height - $top - $bottom
$xMin = -50.0
$xMax = 50.0
$allValues = @($oldData.no_unload_hold_thickness_um) +
    @($newData.D900_refinedZ_no_unload_10pct_hold_thickness_um)
$minimum = ($allValues | ForEach-Object { [double]$_ } | Measure-Object -Minimum).Minimum
$maximum = ($allValues | ForEach-Object { [double]$_ } | Measure-Object -Maximum).Maximum
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
$bluePen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(30, 90, 180), 2.5)
$redPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(220, 45, 45), 3)
$blackBrush = [System.Drawing.Brushes]::Black
$blueBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(30, 90, 180))
$redBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(220, 45, 45))

for ($x = -50; $x -le 50; $x += 10) {
    $px = [single](To-X $x)
    $graphics.DrawLine($gridPen, $px, $top, $px, $top + $plotHeight)
    $label = $x.ToString('0')
    $size = $graphics.MeasureString($label, $smallFont)
    $graphics.DrawString($label, $smallFont, $blackBrush,
        $px - $size.Width / 2, $top + $plotHeight + 10)
}

$tickStart = [Math]::Ceiling($yMin * 5.0) / 5.0
for ($y = $tickStart; $y -le $yMax + 1.0e-8; $y += 0.2) {
    $py = [single](To-Y $y)
    $graphics.DrawLine($gridPen, $left, $py, $left + $plotWidth, $py)
    $label = $y.ToString('0.0')
    $size = $graphics.MeasureString($label, $smallFont)
    $graphics.DrawString($label, $smallFont, $blackBrush,
        $left - $size.Width - 10, $py - $size.Height / 2)
}

$graphics.DrawRectangle($axisPen, $left, $top, $plotWidth, $plotHeight)

for ($i = 1; $i -lt $oldData.Count; $i++) {
    $a = $oldData[$i - 1]
    $b = $oldData[$i]
    $graphics.DrawLine($bluePen,
        (To-X ([double]$a.Z_distance_from_center_mm)),
        (To-Y ([double]$a.no_unload_hold_thickness_um)),
        (To-X ([double]$b.Z_distance_from_center_mm)),
        (To-Y ([double]$b.no_unload_hold_thickness_um)))
}

for ($i = 1; $i -lt $newData.Count; $i++) {
    $a = $newData[$i - 1]
    $b = $newData[$i]
    $graphics.DrawLine($redPen,
        (To-X ([double]$a.Z_distance_from_center_mm)),
        (To-Y ([double]$a.D900_refinedZ_no_unload_10pct_hold_thickness_um)),
        (To-X ([double]$b.Z_distance_from_center_mm)),
        (To-Y ([double]$b.D900_refinedZ_no_unload_10pct_hold_thickness_um)))
}

for ($i = 0; $i -lt $oldData.Count; $i += 5) {
    $row = $oldData[$i]
    $x = [single](To-X ([double]$row.Z_distance_from_center_mm))
    $y = [single](To-Y ([double]$row.no_unload_hold_thickness_um))
    $graphics.FillRectangle($blueBrush, $x - 3, $y - 3, 6, 6)
}
for ($i = 0; $i -lt $newData.Count; $i += 10) {
    $row = $newData[$i]
    $x = [single](To-X ([double]$row.Z_distance_from_center_mm))
    $y = [single](To-Y ([double]$row.D900_refinedZ_no_unload_10pct_hold_thickness_um))
    $graphics.FillEllipse($redBrush, $x - 3, $y - 3, 6, 6)
}

$title = 'D900 thickness profile: original vs refined Z mesh'
$titleSize = $graphics.MeasureString($title, $titleFont)
$graphics.DrawString($title, $titleFont, $blackBrush,
    ($width - $titleSize.Width) / 2, 15)
$xTitle = 'Distance from film center Z (mm)'
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
$graphics.DrawLine($bluePen, $legendX, $legendY, $legendX + 42, $legendY)
$graphics.DrawString('Original: film Z 1 mm, roll Z 10 mm', $font, $blackBrush,
    $legendX + 52, $legendY - 10)
$graphics.DrawLine($redPen, $legendX, $legendY + 32, $legendX + 42, $legendY + 32)
$graphics.DrawString('Refined: film Z 0.5 mm, roll Z 2 mm', $font, $blackBrush,
    $legendX + 52, $legendY + 22)

$bitmap.Save($outPng, [System.Drawing.Imaging.ImageFormat]::Png)
$graphics.Dispose()
$bitmap.Dispose()
$font.Dispose()
$smallFont.Dispose()
$titleFont.Dispose()
$axisPen.Dispose()
$gridPen.Dispose()
$bluePen.Dispose()
$redPen.Dispose()
$blueBrush.Dispose()
$redBrush.Dispose()

Write-Output $outCsv
Write-Output $outPng
