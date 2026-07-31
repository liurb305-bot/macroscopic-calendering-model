param(
    [string]$WorkDir = 'E:\abaqus\3Dfuxian'
)

Add-Type -AssemblyName System.Drawing

$csv10 = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_thickness_profile_Z.csv'
$csv20 = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_20pct_thickness_profile_Z.csv'
$outCsv = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_10pct_vs_20pct_thickness_profile_Z.csv'
$outPng = Join-Path $WorkDir 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_10pct_vs_20pct_thickness_profile_Z.png'

$data10 = Import-Csv -LiteralPath $csv10
$data20 = Import-Csv -LiteralPath $csv20
$map20 = @{}
foreach ($row in $data20) {
    $map20[[double]$row.Z_distance_from_center_mm] = [double]$row.no_unload_20pct_hold_thickness_um
}

$comparison = foreach ($row in $data10) {
    $z = [double]$row.Z_distance_from_center_mm
    [pscustomobject]@{
        Z_distance_from_center_mm = $z
        initial_thickness_um = 150.0
        hold_10pct_thickness_um = [double]$row.no_unload_hold_thickness_um
        hold_20pct_thickness_um = $map20[$z]
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
$allThickness = @($comparison.hold_10pct_thickness_um) + @($comparison.hold_20pct_thickness_um)
$yMin = [Math]::Floor((($allThickness | Measure-Object -Minimum).Minimum) - 0.5)
$yMax = 150.5

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
$blackPen = New-Object System.Drawing.Pen([System.Drawing.Color]::Black, 2)
$gridPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(225, 225, 225), 1)
$grayPen = New-Object System.Drawing.Pen([System.Drawing.Color]::Gray, 2)
$grayPen.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
$redPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(220, 45, 45), 3)
$bluePen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(30, 90, 180), 3)
$blackBrush = [System.Drawing.Brushes]::Black
$redBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(220, 45, 45))
$blueBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(30, 90, 180))

for ($x = -50; $x -le 50; $x += 10) {
    $px = [single](To-X $x)
    $graphics.DrawLine($gridPen, $px, $top, $px, $top + $plotHeight)
    $label = $x.ToString('0')
    $size = $graphics.MeasureString($label, $smallFont)
    $graphics.DrawString($label, $smallFont, $blackBrush, $px - $size.Width / 2, $top + $plotHeight + 10)
}

for ($y = [Math]::Ceiling($yMin); $y -le 150; $y += 1) {
    $py = [single](To-Y $y)
    $graphics.DrawLine($gridPen, $left, $py, $left + $plotWidth, $py)
    $label = $y.ToString('0')
    $size = $graphics.MeasureString($label, $smallFont)
    $graphics.DrawString($label, $smallFont, $blackBrush, $left - $size.Width - 10, $py - $size.Height / 2)
}

$graphics.DrawRectangle($blackPen, $left, $top, $plotWidth, $plotHeight)
$graphics.DrawLine($grayPen, (To-X $xMin), (To-Y 150.0), (To-X $xMax), (To-Y 150.0))

for ($i = 1; $i -lt $comparison.Count; $i++) {
    $a = $comparison[$i - 1]
    $b = $comparison[$i]
    $graphics.DrawLine($bluePen,
        (To-X $a.Z_distance_from_center_mm), (To-Y $a.hold_10pct_thickness_um),
        (To-X $b.Z_distance_from_center_mm), (To-Y $b.hold_10pct_thickness_um))
    $graphics.DrawLine($redPen,
        (To-X $a.Z_distance_from_center_mm), (To-Y $a.hold_20pct_thickness_um),
        (To-X $b.Z_distance_from_center_mm), (To-Y $b.hold_20pct_thickness_um))
}

for ($i = 0; $i -lt $comparison.Count; $i += 5) {
    $row = $comparison[$i]
    $x = [single](To-X $row.Z_distance_from_center_mm)
    $y10 = [single](To-Y $row.hold_10pct_thickness_um)
    $y20 = [single](To-Y $row.hold_20pct_thickness_um)
    $graphics.FillRectangle($blueBrush, $x - 3, $y10 - 3, 6, 6)
    $graphics.FillEllipse($redBrush, $x - 3, $y20 - 3, 6, 6)
}

$title = 'Film thickness under load: 10% vs 20% reduction'
$titleSize = $graphics.MeasureString($title, $titleFont)
$graphics.DrawString($title, $titleFont, $blackBrush, ($width - $titleSize.Width) / 2, 15)
$xTitle = 'Distance from film center Z (mm)'
$xTitleSize = $graphics.MeasureString($xTitle, $font)
$graphics.DrawString($xTitle, $font, $blackBrush, ($width - $xTitleSize.Width) / 2, $height - 43)
$graphics.TranslateTransform(28, $height / 2)
$graphics.RotateTransform(-90)
$yTitle = 'Film thickness (um)'
$yTitleSize = $graphics.MeasureString($yTitle, $font)
$graphics.DrawString($yTitle, $font, $blackBrush, -$yTitleSize.Width / 2, 0)
$graphics.ResetTransform()

$legendX = $left + 25
$legendY = $top + 24
$graphics.DrawLine($bluePen, $legendX, $legendY, $legendX + 42, $legendY)
$graphics.DrawString('10% reduction, Hold', $font, $blackBrush, $legendX + 52, $legendY - 10)
$graphics.DrawLine($redPen, $legendX, $legendY + 30, $legendX + 42, $legendY + 30)
$graphics.DrawString('20% reduction, Hold', $font, $blackBrush, $legendX + 52, $legendY + 20)
$graphics.DrawLine($grayPen, $legendX, $legendY + 60, $legendX + 42, $legendY + 60)
$graphics.DrawString('Initial thickness 150 um', $font, $blackBrush, $legendX + 52, $legendY + 50)

$bitmap.Save($outPng, [System.Drawing.Imaging.ImageFormat]::Png)
$graphics.Dispose()
$bitmap.Dispose()
$font.Dispose()
$smallFont.Dispose()
$titleFont.Dispose()
$blackPen.Dispose()
$gridPen.Dispose()
$grayPen.Dispose()
$redPen.Dispose()
$bluePen.Dispose()
$redBrush.Dispose()
$blueBrush.Dispose()

Write-Output $outCsv
Write-Output $outPng
