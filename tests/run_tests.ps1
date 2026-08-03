#Requires -Version 5.1
[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$RepoDir = Split-Path -Parent $PSScriptRoot
$WorkspaceDir = Split-Path -Parent $RepoDir
$TestBuildDir = Join-Path ([System.IO.Path]::GetTempPath()) "m0g3519-host-tests"
New-Item -ItemType Directory -Force -Path $TestBuildDir | Out-Null

$Includes = Get-ChildItem -Path (Join-Path $RepoDir "src") -Directory -Recurse |
    ForEach-Object { "-I$($_.FullName)" }
$CommonFlags = @("-std=c11", "-Wall", "-Wextra", "-Werror") + $Includes

function Invoke-CTest {
    param([string] $Name, [string[]] $Sources)
    $Output = Join-Path $TestBuildDir "$Name.exe"
    & gcc @CommonFlags @Sources -lm -o $Output
    if ($LASTEXITCODE -ne 0) { throw "$Name compile failed" }
    & $Output
    if ($LASTEXITCODE -ne 0) { throw "$Name failed" }
}

Push-Location $RepoDir
try {
    Invoke-CTest "test_ball_motion_profile" @(
        "tests/test_ball_motion_profile.c", "src/middle/ball_motion_profile.c")
    Invoke-CTest "test_balance_actuator_trajectory" @(
        "tests/test_balance_actuator_trajectory.c",
        "src/middle/balance_actuator_trajectory.c")
    Invoke-CTest "test_balance_control" @(
        "tests/test_balance_control.c", "src/middle/balance_control.c")
    Invoke-CTest "test_balance_closed_loop" @(
        "tests/test_balance_closed_loop.c", "src/middle/balance_control.c",
        "src/middle/ball_motion_profile.c",
        "src/middle/balance_actuator_trajectory.c")
    Invoke-CTest "test_balance_app" @(
        "tests/test_balance_app.c", "src/app/balance_app.c",
        "src/middle/balance_control.c", "src/middle/ball_motion_profile.c",
        "src/middle/balance_actuator_trajectory.c",
        "src/middle/balance_linkage.c")
    Invoke-CTest "test_emm42_demo_app" @(
        "tests/test_emm42_demo_app.c", "src/app/emm42_demo_app.c",
        "src/middle/balance_linkage.c")
    Invoke-CTest "test_ball_return_demo_app" @(
        "tests/test_ball_return_demo_app.c",
        "src/app/ball_return_demo_app.c",
        "src/middle/ball_motion_profile.c",
        "src/middle/balance_actuator_trajectory.c",
        "src/middle/balance_linkage.c")

    $FastDemoObject = Join-Path $TestBuildDir "ball_return_demo_app_fast.o"
    & gcc @CommonFlags "-DBALL_RETURN_DEMO_SPEED_SCALE=1.5f" `
        -c "src/app/ball_return_demo_app.c" -o $FastDemoObject
    if ($LASTEXITCODE -ne 0) { throw "scaled ball return demo compile failed" }

    $ModeBuilds = @(
        @{
            Name = "ball_return"
            Defines = @("-DBALL_RETURN_DEMO_ENABLE=1",
                        "-DEMM42_BALANCE_DEMO_ENABLE=0",
                        "-DBALANCE_CONTROL_ENABLE=0")
        },
        @{
            Name = "calibration_v5"
            Defines = @("-DBALL_RETURN_DEMO_ENABLE=0",
                        "-DEMM42_BALANCE_DEMO_ENABLE=1",
                        "-DBALANCE_CONTROL_ENABLE=0")
        },
        @{
            Name = "balance_control"
            Defines = @("-DBALL_RETURN_DEMO_ENABLE=0",
                        "-DEMM42_BALANCE_DEMO_ENABLE=0",
                        "-DBALANCE_CONTROL_ENABLE=1")
        }
    )
    foreach ($Mode in $ModeBuilds) {
        foreach ($Source in @("src/main.c", "src/app/uart3_maix_app.c")) {
            $SourceName = [System.IO.Path]::GetFileNameWithoutExtension($Source)
            $Object = Join-Path $TestBuildDir "$($SourceName)_$($Mode.Name).o"
            & gcc @CommonFlags @($Mode.Defines) -c $Source -o $Object
            if ($LASTEXITCODE -ne 0) {
                throw "$($Mode.Name) $Source compile failed"
            }
        }
    }

    & python (Join-Path $WorkspaceDir "tools/uart_log/test_mcu_telemetry_stream.py")
    if ($LASTEXITCODE -ne 0) { throw "telemetry Python tests failed" }
    & python (Join-Path $WorkspaceDir "tools/uart_log/test_analyze_ball_dynamics.py")
    if ($LASTEXITCODE -ne 0) { throw "ball dynamics analyzer tests failed" }
}
finally {
    Pop-Location
}

Write-Host "All host tests passed"
