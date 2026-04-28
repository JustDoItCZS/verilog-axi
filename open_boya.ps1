param(
    [string]$NodeId,
    [string]$WavePath,
    [string]$BoYaExe = "D:\project\codex\BoYa\bin\BoYa.exe",
    [string]$RepoRoot = (Resolve-Path ".").Path,
    [switch]$Restart,
    [ValidateSet("sources", "filelist")][string]$DesignArgMode = "sources"
)

$ErrorActionPreference = "Stop"

function Resolve-WaveFromNodeId {
    param(
        [Parameter(Mandatory = $true)][string]$InputNodeId,
        [Parameter(Mandatory = $true)][string]$RootDir
    )

    $parts = $InputNodeId -split "::"
    if ($parts.Count -lt 2) {
        throw "Invalid NodeId. Example: tb\axil_ram\test_axil_ram.py::test_axil_ram[8]"
    }

    $testFile = $parts[0]
    $testName = $parts[1]
    $simName = $testName.Replace("[", "-").Replace("]", "")

    $testFilePath = Join-Path $RootDir $testFile
    if (-not (Test-Path $testFilePath)) {
        throw "Test file not found: $testFilePath"
    }

    $testDir = Split-Path -Parent $testFilePath
    $simDir = Join-Path $testDir ("sim_build\" + $simName)

    if (-not (Test-Path $simDir)) {
        throw "sim_build directory not found: $simDir"
    }

    $fst = Get-ChildItem -Path $simDir -File -Filter *.fst |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1

    if (-not $fst) {
        throw "No FST waveform found under: $simDir"
    }

    return $fst.FullName
}

if (-not (Test-Path $BoYaExe)) {
    throw "BoYa executable not found: $BoYaExe"
}

$repoResolved = (Resolve-Path $RepoRoot).Path
$waveResolved = $null

if ($WavePath) {
    $waveResolved = (Resolve-Path $WavePath).Path
} elseif ($NodeId) {
    $waveResolved = Resolve-WaveFromNodeId -InputNodeId $NodeId -RootDir $repoResolved
} else {
    $latest = Get-ChildItem -Path (Join-Path $repoResolved "tb") -Recurse -File -Filter *.fst |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if (-not $latest) {
        throw "No FST waveform found under tb. Run simulation first."
    }
    $waveResolved = $latest.FullName
}

$waveDir = Split-Path -Parent $waveResolved
$filelistPath = Join-Path $waveDir "boya_sources.f"
$waveInfo = Get-Item -LiteralPath $waveResolved
if ($waveInfo.Length -lt 64) {
    throw "Waveform file is too small, likely invalid: $waveResolved"
}

# Bind current workspace RTL to the selected waveform.
# For strict historical mapping, run this at the matching git commit.
$rtlFiles = Get-ChildItem -Path (Join-Path $repoResolved "rtl") -Recurse -File -Include *.v,*.sv |
    Sort-Object FullName |
    ForEach-Object { $_.FullName }

if (-not $rtlFiles -or $rtlFiles.Count -eq 0) {
    throw "No RTL source files found under rtl."
}

Set-Content -Path $filelistPath -Value $rtlFiles -Encoding ASCII

Write-Host "Wave:     $waveResolved"
Write-Host "Filelist: $filelistPath"
Write-Host "Sources:  $($rtlFiles.Count) files"
Write-Host "Mode:     $DesignArgMode"

if ($Restart) {
    Get-Process -Name BoYa -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Milliseconds 300
}

if ($DesignArgMode -eq "sources") {
    $args = @("-w", $waveResolved) + $rtlFiles
    Start-Process -FilePath $BoYaExe -ArgumentList $args
} else {
    Start-Process -FilePath $BoYaExe -ArgumentList @("-w", $waveResolved, "-f", $filelistPath)
}
