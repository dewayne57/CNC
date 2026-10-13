param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$originalPath = $env:PATH
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compilerPath) + [System.IO.Path]::PathSeparator + $env:PATH
$sourceDir = Split-Path -Parent $PSScriptRoot
$testExe = Join-Path ([System.IO.Path]::GetTempPath()) ('cnc-motion-tests-' + [guid]::NewGuid().ToString('N') + '.exe')
$sources = @('GCodeParser.cpp', 'MotionProfile.cpp', 'Motion.cpp', 'ArcMotion.cpp', 'Homing.cpp', 'GantryHoming.cpp', 'XGantry.cpp', 'EncoderFeedback.cpp', 'Spindle.cpp', 'Display.cpp', 'Executor.cpp') |
    ForEach-Object { Join-Path $sourceDir $_ }
try {
    # Fixture: 1.8-degree motors, 16 microsteps, 4:1 gears, 160 mm/axle rev = 80 steps/mm.
    & $Compiler '-std=c++11' '-Wall' '-Wextra' '-Werror' '-O2' '-DCNC_X_MICROSTEPS=16' '-DCNC_X_AXLE_TRAVEL_MM=160.0f' "-I$PSScriptRoot" "-I$sourceDir" (Join-Path $PSScriptRoot 'test_main.cpp') @sources '-o' $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
    foreach ($scenario in @('all', 'feedback', 'hardlimit')) {
        & $testExe $scenario
        if ($LASTEXITCODE -ne 0) { throw "Tests failed: $scenario" }
    }
    & $Compiler '-std=c++11' '-Wall' '-Wextra' '-Werror' '-O2' '-DCNC_X_MICROSTEPS=0' '-DCNC_X_AXLE_TRAVEL_MM=0.0f' "-I$PSScriptRoot" "-I$sourceDir" (Join-Path $PSScriptRoot 'test_main.cpp') @sources '-o' $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Unconfigured test compilation failed' }
    & $testExe 'unconfigured'
    if ($LASTEXITCODE -ne 0) { throw 'Unconfigured tests failed' }
} finally {
    $env:PATH = $originalPath
    if (Test-Path -LiteralPath $testExe) { Remove-Item -LiteralPath $testExe }
}
