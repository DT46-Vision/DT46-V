param([string]$Frames='outputs/vision_accuracy_20261030/translate-noangle-seed42-20261030-030430/5_off/vision/frames.jsonl',[string[]]$Candidates=@('baseline','q03'))
$ErrorActionPreference='Stop'
$visionRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
. (Join-Path $visionRoot '../.local/runtime-path-check/activate.ps1')
$env:PATH=(Join-Path $visionRoot 'build/aim-validation-full/standalone')+';'+$env:PATH
$source=Get-Content (Join-Path $visionRoot 'src/rm_tracker/config/tracker_params_infantry.yaml') -Raw
$values=@{baseline=@(0.1,0.5);lateral=@(0.1,0.05);responsive=@(0.5,0.05);fast=@(1.0,0.05);q03=@(0.3,0.05);q04=@(0.4,0.05);r01=@(0.5,0.1)}
$batch=Join-Path $PSScriptRoot ('replay-'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
foreach($name in $Candidates){
    if(!$values.ContainsKey($name)){throw "Unknown candidate: $name"}
    $folder=Join-Path $batch $name
    New-Item -ItemType Directory -Path $folder -Force | Out-Null
    $q,$r=$values[$name]
    $yaml=$source -replace 'ekf_QR_q_xyz: [0-9.]+',("ekf_QR_q_xyz: $q") -replace 'ekf_QR_r_xyz_factor: [0-9.]+',("ekf_QR_r_xyz_factor: $r")
    [IO.File]::WriteAllText("$folder/tracker.yaml",$yaml,[Text.UTF8Encoding]::new($false))
    $parameters=Get-Content (Join-Path $visionRoot 'standalone/virtual_camera.json') -Raw | ConvertFrom-Json
    $parameters.tracker | Add-Member -NotePropertyName ekf_QR_q_xyz -NotePropertyValue $q -Force
    $parameters.tracker | Add-Member -NotePropertyName ekf_QR_r_xyz_factor -NotePropertyValue $r -Force
    [IO.File]::WriteAllText("$folder/parameters.json",($parameters | ConvertTo-Json -Depth 10),[Text.UTF8Encoding]::new($false))
    & (Join-Path $visionRoot 'build/aim-validation-full/dt46_tracker_log_check.exe') $Frames "$folder/tracker.yaml" $folder
    if($LASTEXITCODE-ne0){throw "Replay failed: $name"}
    & (Join-Path $visionRoot 'build/aim-validation-full/dt46_tracker_switch_check.exe') "$folder/tracker.yaml" "$folder/switch"
    [IO.File]::WriteAllText("$folder/switch-exit.json",("{`"exit_code`":$LASTEXITCODE}"),[Text.UTF8Encoding]::new($false))
}
Write-Output "BATCH=$batch"
