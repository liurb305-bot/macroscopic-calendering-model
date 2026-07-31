param(
    [string]$WorkDir = 'E:\abaqus\3Dfuxian'
)

Add-Type -AssemblyName System.Drawing

$csv10 = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D900_10pct_RefinedZ_thickness_profile_X.csv'
$csv20 = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_20pct_thickness_profile_X.csv'
$outCsv = Join-Path $WorkDir 'D900_10pct_vs_20pct_thickness_profile_X.csv'
$outPng = Join-Path $WorkDir 'D900_10pct_vs_20pct_thickness_profile_X.png'

$data10 = Import-Csv -LiteralPath $csv10
$data20 = Import-Csv -LiteralPath $csv20
$map20 = @{}
foreach ($row in $data20) {
    $map20[[double]$row.X_distance_from_deformation_center_mm] =
        [double]$row.D900_20pct_hold_thickness_um
}

$comparison = foreach ($row in $data10) {
    $x = [double]$row.X_distance_from_deformation_center_mm
    $value10 = [double]$row.D900_refinedZ_10pct_hold_thickness_um
    $value20 = $map20[$x]
    [pscustomobject]@{
        X_distance_from_deformation_center_mm = $x
        initial_thickness_um = 150.0
        D900_10pct_hold_thickness_um = $value10
        D900_20pct_hold_thickness_um = $value20
        thickness_20pct_minus_10pct_um = $value20 - $value10
        compression_10pct_um = 150.0 - $value10
        compression_20pct_um = 150.0 - $value20
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
$xMin = -5.0
$xMax = 5.0
$allValues = @($comparison.D900_10pct_hold_thickness_um) +
    @($comparison.D900_20pct_hold_thickness_um)
$minimum = ($allValues | Measure-Object -Minimum).Minimum
$maximum = ($allValues | Measure-Object -Maximum).Maximum
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
$bluePen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(30, 90, 180), 3)
$redPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(220, 45, 45), 3)
$grayPen = New-Object System.Drawing.Pen([System.Drawing.Color]::Gray, 2)
$grayPen.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
$blackBrush = [System.Drawing.Brushes]::Black
$blueBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(30, 90, 180))
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

for ($i = 1; $i -lt $comparison.Count; $i++) {
    $a = $comparison[$i - 1]
    $b = $comparison[$i]
    $graphics.DrawLine($bluePen,
        (To-X $a.X_distance_from_deformation_center_mm),
        (To-Y $a.D900_10pct_hold_thickness_um),
        (To-X $b.X_distance_from_deformation_center_mm),
        (To-Y $b.D900_10pct_hold_thickness_um))
    $graphics.DrawLine($redPen,
        (To-X $a.X_distance_from_deformation_center_mm),
        (To-Y $a.D900_20pct_hold_thickness_um),
        (To-X $b.X_distance_from_deformation_center_mm),
        (To-Y $b.D900_20pct_hold_thickness_um))
}

for ($i = 0; $i -lt $comparison.Count; $i += 5) {
    $row = $comparison[$i]
    $x = [single](To-X $row.X_distance_from_deformation_center_mm)
    $y10 = [single](To-Y $row.D900_10pct_hold_thickness_um)
    $y20 = [single](To-Y $row.D900_20pct_hold_thickness_um)
    $graphics.FillRectangle($blueBrush, $x - 3, $y10 - 3, 6, 6)
    $graphics.FillEllipse($redBrush, $x - 3, $y20 - 3, 6, 6)
}

$title = 'D900 thickness along X: 10% vs 20% reduction'
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
$graphics.DrawLine($bluePen, $legendX, $legendY, $legendX + 42, $legendY)
$graphics.DrawString('10% reduction, Hold (Refined Z mesh)', $font, $blackBrush,
    $legendX + 52, $legendY - 10)
$graphics.DrawLine($redPen, $legendX, $legendY + 32, $legendX + 42, $legendY + 32)
$graphics.DrawString('20% reduction, Hold (original Z mesh)', $font, $blackBrush,
    $legendX + 52, $legendY + 22)
$graphics.DrawLine($grayPen, $legendX, $legendY + 64, $legendX + 42, $legendY + 64)
$graphics.DrawString('Initial thickness 150 um', $font, $blackBrush,
    $legendX + 52, $legendY + 54)

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
$grayPen.Dispose()
$blueBrush.Dispose()
$redBrush.Dispose()

Write-Output $outCsv
Write-Output $outPng
