param([double[]]$Distances=@(1.4,3,5,7),[double[]]$YawDegrees=@(0,-30,30,-60,60),[int]$Seconds=5)
$ErrorActionPreference='Stop'
$visionRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$simRoot=(Resolve-Path (Join-Path $visionRoot '..')).Path
. (Join-Path $simRoot '.local/runtime-path-check/activate.ps1')
$env:PATH=(Join-Path $visionRoot 'build/aim-validation-full/standalone')+';'+$env:PATH
$sim=Join-Path $simRoot 'build/validation/runtime-path-check/rm_sim.exe'
$vision=Join-Path $visionRoot 'build/aim-validation-full/dt46_vision.exe'
$batch=Join-Path $PSScriptRoot ('scan-'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $batch | Out-Null
foreach($distance in $Distances){foreach($yaw in $YawDegrees){
    $run=Join-Path $batch ("d$distance-y$yaw");New-Item -ItemType Directory -Path $run | Out-Null
    $plan=Get-Content (Join-Path $simRoot 'config/sessions/aim_tracking.json') -Raw | ConvertFrom-Json
    $robots=Get-Content (Join-Path $simRoot 'config/robots/robots_aim_tracking.json') -Raw | ConvertFrom-Json
    $robots[1].position_m[1]=-3.4+$distance;$robots[1].yaw_deg=$yaw
    [IO.File]::WriteAllText("$run/robots.json",($robots|ConvertTo-Json -Depth 12),[Text.UTF8Encoding]::new($false))
    $plan.world.robots="$run/robots.json";$plan.duration_s=$Seconds
    $plan.vehicles.infantry3.decision=[pscustomobject]@{type='script';aim=$true;fire=$false}
    $plan.vehicles.target3.decision.translation_mode='stationary';$plan.vehicles.target3.decision.rotation_mode='off'
    [IO.File]::WriteAllText("$run/scenario.json",($plan|ConvertTo-Json -Depth 16),[Text.UTF8Encoding]::new($false))
    [IO.File]::WriteAllText("$run/stop.json",'{"op":"stop"}',[Text.UTF8Encoding]::new($false))
    $hostRun=$null;$visionRun=$null
    try{
        $hostRun=Start-Process -FilePath $sim -ArgumentList @('--root',$simRoot,'--session',"$run/scenario.json",'--output',"$run/session",'--hidden','--no-vision','--no-logs','--log','reports','--log','camera_truth') -WorkingDirectory $simRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput "$run/host.stdout.log" -RedirectStandardError "$run/host.stderr.log"
        $deadline=(Get-Date).AddSeconds(20)
        while(!(Test-Path "$run/session/cameras/0/camera.json") -and (Get-Date)-lt$deadline -and !$hostRun.HasExited){Start-Sleep -Milliseconds 100}
        if(!(Test-Path "$run/session/cameras/0/camera.json")){throw 'Camera did not start'}
        $visionRun=Start-Process -FilePath $vision -ArgumentList @('--root',$visionRoot,'--camera-session',"$run/session/cameras/0/camera.json",'--debug-session',"$run/session/cameras/0/debug.json",'--output',"$run/vision",'--debug','--debug-hidden') -WorkingDirectory $visionRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput "$run/vision.stdout.log" -RedirectStandardError "$run/vision.stderr.log"
        $began=Get-Date;$saved=$false
        while(!$hostRun.HasExited -and (Get-Date)-lt$began.AddSeconds($Seconds+20)){
            if(!$saved -and (Get-Date)-gt$began.AddSeconds(2.5)){
                & $vision --save-debug --debug-session "$run/session/cameras/0/debug.json" *> "$run/save.log";$saved=$true
            }
            Start-Sleep -Milliseconds 100
        }
        if(!$hostRun.HasExited -or !$visionRun.WaitForExit(5000)){throw 'Scan process timeout'}
        if($hostRun.ExitCode-ne0 -or $visionRun.ExitCode-ne0){throw 'Scan process failed'}
        $report=Get-Content "$run/session/report.json" -Raw|ConvertFrom-Json
        if($report.metrics.projectiles.shot_count-ne0){throw 'Unexpected shot in static scan'}
        Write-Output ("DONE d=$distance yaw=$yaw")
    }finally{
        if($hostRun -and !$hostRun.HasExited){& (Join-Path $simRoot 'build/validation/runtime-path-check/sim_session.exe') --connect "$run/session/session.json" --request "$run/stop.json" *> "$run/stop.log";if(!$hostRun.WaitForExit(5000)){Stop-Process -Id $hostRun.Id}}
        if($visionRun -and !$visionRun.HasExited){if(!$visionRun.WaitForExit(5000)){Stop-Process -Id $visionRun.Id}}
    }
}}
Write-Output "BATCH=$batch"
