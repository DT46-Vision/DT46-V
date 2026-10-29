param([string]$Batch='baseline',[string[]]$Cases=@('1.4_off','3_off','5_off','1.4_constant','3_constant','5_constant'),[int]$Seconds=12,[switch]$Gui,[string]$Translation='stationary')
$ErrorActionPreference='Stop'
$visionRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$simRoot=(Resolve-Path (Join-Path $visionRoot '..')).Path
. (Join-Path $simRoot '.local/runtime-path-check/activate.ps1')
$env:PATH=(Join-Path $visionRoot 'build/aim-validation-full/standalone')+';'+$env:PATH
$sim=Join-Path $simRoot 'build/validation/runtime-path-check/rm_sim.exe'
$sessionTool=Join-Path $simRoot 'build/validation/runtime-path-check/sim_session.exe'
$vision=Join-Path $visionRoot 'build/aim-validation-full/dt46_vision.exe'
$batch=Join-Path $PSScriptRoot ($Batch+'-'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $batch | Out-Null
$identities=@($sim,$vision,(Join-Path $visionRoot 'build/aim-validation-full/standalone/dt46_aim.dll'),(Join-Path $visionRoot 'src/rm_tracker/config/tracker_params_infantry.yaml'),(Join-Path $visionRoot 'src/rm_detector/config/detector_params_infantry.yaml')) | ForEach-Object {Get-FileHash -LiteralPath $_ -Algorithm SHA256 | Select-Object Path,Hash}
$identities | ConvertTo-Json | Set-Content "$batch/identities.json" -Encoding utf8
$results=@()
foreach($case in $Cases){
    $parts=$case.Split('_');if($parts.Count-ne2){throw 'case must be distance_rotation'}
    $run=Join-Path $batch $case;New-Item -ItemType Directory -Path $run | Out-Null
    $plan=Get-Content (Join-Path $simRoot 'config/sessions/aim_tracking.json') -Raw | ConvertFrom-Json
    $plan.referee=Get-Content (Join-Path $simRoot 'config/referee/referee.json') -Raw | ConvertFrom-Json
    $plan.referee.enabled=$true
    foreach($rule in $plan.referee.rules.PSObject.Properties){$rule.Value=($rule.Name-eq'armor_damage')}
    $plan.referee.vehicles=[pscustomobject]@{infantry3=[pscustomobject]@{profile='infantry_hp_cooling'};target3=[pscustomobject]@{profile='infantry_hp_cooling'}}
    $plan.referee.performance_profiles.infantry_hp_cooling.max_hp=100000
    $robots=Get-Content (Join-Path $simRoot 'config/robots/robots_aim_tracking.json') -Raw | ConvertFrom-Json
    $robots[1].position_m[1]=-3.4+[double]$parts[0]
    $robots | ConvertTo-Json -Depth 12 | Set-Content "$run/robots.json" -Encoding utf8
    $plan.world.robots="$run/robots.json"
    $plan.name='vision-tracking-validation-'+$case
    $plan.experiment.baseline='Current vision branch; actual rendered RGB, detector and PnP; HP damage enabled.'
    $plan.vehicles.infantry3.decision=[pscustomobject]@{type='script';aim=$true;fire=$true}
    $plan.vehicles.target3.decision.translation_mode=$Translation;$plan.vehicles.target3.decision.rotation_mode=$parts[1]
    if($Translation-ne'stationary'){$plan.vehicles.target3.decision.axis=@(1.0,0.0);$plan.vehicles.target3.decision.amplitude_m=0.75}
    $plan.duration_s=$Seconds
    [IO.File]::WriteAllText((Join-Path $run 'scenario.json'),($plan | ConvertTo-Json -Depth 20),[Text.UTF8Encoding]::new($false))
    [IO.File]::WriteAllText((Join-Path $run 'stop.json'),'{"op":"stop"}',[Text.UTF8Encoding]::new($false))
    $hostRun=$null;$visionRun=$null
    try{
        $hostRun=Start-Process -FilePath $sim -ArgumentList @('--root',$simRoot,'--session',"$run/scenario.json",'--output',"$run/session",'--hidden','--no-vision','--no-logs','--log','events','--log','reports','--log','camera_truth') -WorkingDirectory $simRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput "$run/host.stdout.log" -RedirectStandardError "$run/host.stderr.log"
        $deadline=(Get-Date).AddSeconds(20)
        while(!(Test-Path "$run/session/cameras/0/camera.json") -and (Get-Date)-lt$deadline -and !$hostRun.HasExited){Start-Sleep -Milliseconds 100}
        if(!(Test-Path "$run/session/cameras/0/camera.json")){throw "camera did not start: $run"}
        $arguments=@('--root',$visionRoot,'--camera-session',"$run/session/cameras/0/camera.json",'--debug-session',"$run/session/cameras/0/debug.json",'--output',"$run/vision")
        if($Gui){$arguments+='--debug'}
        $visionRun=Start-Process -FilePath $vision -ArgumentList $arguments -WorkingDirectory $visionRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput "$run/vision.stdout.log" -RedirectStandardError "$run/vision.stderr.log"
        $saved=$false;$began=Get-Date
        while(!$hostRun.HasExited -and (Get-Date)-lt$began.AddSeconds($Seconds+25)){
            if($Gui -and !$saved -and (Get-Date)-gt$began.AddSeconds(1.2) -and (Test-Path "$run/session/cameras/0/debug.json")){
                & $vision --save-debug --debug-session "$run/session/cameras/0/debug.json" *> "$run/save-debug.log";$saved=$true
            }
            Start-Sleep -Milliseconds 200
        }
        if(!$hostRun.HasExited){throw "host timeout: $run"}
        if(!$visionRun.WaitForExit(5000)){throw "vision did not finish: $run"}
        $report=Get-Content "$run/session/report.json" -Raw | ConvertFrom-Json
        $frames=0;$tracking=0;$fire=0;$invalid_fire=0;$model_frames=0;$reasons=@{}
        foreach($line in [IO.File]::ReadLines("$run/vision/frames.jsonl")){
            $frame=$line | ConvertFrom-Json;$frames++;if($frame.state-eq'TRACKING'){$tracking++};if($frame.can_fire){$fire++}
            if($frame.can_fire -and (!$frame.aim.valid -or $frame.state-ne'TRACKING' -or @($frame.tracker_debug.fire_blockers).Count-gt0 -or $frame.tracker_debug.observed_plate-lt0)){$invalid_fire++}
            if(@($frame.tracker_debug.body_plates).Count-eq4){$model_frames++}
            foreach($reason in $frame.tracker_debug.fire_blockers){$reasons[$reason]++}
        }
        $window_errors=@([IO.File]::ReadLines("$run/vision/debug_events.jsonl") | Where-Object {($_ | ConvertFrom-Json).event-eq'window_error'}).Count
        $result=[ordered]@{case=$case;run=$run;frames=$frames;tracking=$tracking;fire_frames=$fire;invalid_fire_frames=$invalid_fire;model_frames=$model_frames;window_errors=$window_errors;shots=$report.metrics.projectiles.shot_count;hits=$report.metrics.projectiles.hit_count;target_hp=$report.referee.vehicles.target3.hp;host_exit=$hostRun.ExitCode;vision_exit=$visionRun.ExitCode;blockers=$reasons}
        $result.passed=($result.shots-ge2 -and $result.target_hp-lt100000 -and $result.host_exit-eq0 -and $result.vision_exit-eq0 -and $invalid_fire-eq0 -and $window_errors-eq0 -and $model_frames-gt0)
        $result | ConvertTo-Json -Depth 5 | Set-Content "$run/checks.json" -Encoding utf8
        $results+=[pscustomobject]$result
        $result | ConvertTo-Json -Depth 5 -Compress
    }finally{
        if($hostRun -and !$hostRun.HasExited){& $sessionTool --connect "$run/session/session.json" --request "$run/stop.json" *> "$run/stop.log";if(!$hostRun.WaitForExit(5000)){Stop-Process -Id $hostRun.Id}}
        if($visionRun -and !$visionRun.HasExited){if(!$visionRun.WaitForExit(5000)){Stop-Process -Id $visionRun.Id}}
    }
}
$results | Select-Object case,frames,tracking,fire_frames,shots,hits,target_hp,host_exit,vision_exit,passed | Export-Csv "$batch/summary.csv" -NoTypeInformation -Encoding utf8
Write-Output "BATCH=$batch"
if(@($results | Where-Object {!$_.passed}).Count){throw 'One or more live cases failed; see preserved checks.json'}
