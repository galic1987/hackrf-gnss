#!/usr/bin/env bash
# metrology_suite_daemon.sh — Unified manager for all 30 GNSS metrology and space weather daemons.

WORKDIR="/Volumes/Radiator 8TB/gnss/hackrf_gnss"
OBS="/Volumes/Radiator 8TB/gnss/observations"
LOGDIR="/tmp/metrology_daemons"
mkdir -p "$LOGDIR"

DAEMONS=(
    "carrier_single_difference_engine.py --interval 2.0"
    "solar_dawn_detector.py --interval 2.0"
    "gnss_reflectometry_sounder.py --interval 2.0"
    "hoi_refraction_engine.py --loop --interval 2.0"
    "hatch_divergence_sounder.py --loop --interval 2.0"
    "rf_link_budget_radiometer.py --loop --interval 2.0"
    "gnss_meteorology_pwv.py --loop --interval 2.0"
    "gdop_error_ellipsoid_analyzer.py --loop --interval 2.0"
    "relativistic_space_time_inspector.py --loop --interval 2.0"
    "tropo_saastamoinen_model.py --loop --interval 2.0"
    "sidereal_multipath_analyzer.py --loop --interval 2.0"
    "isb_adev_analyzer.py --loop --interval 2.0"
    "thermal_phase_analyzer.py --loop --interval 2.0"
    "iono_klobuchar_benchmark.py --loop --interval 2.0"
    "iono_tid_analyzer.py --loop --interval 2.0"
    "iono_scintillation.py --loop --interval 2.0"
    "carrier_double_difference_engine.py --interval 2.0"
    "post_sunrise_flux_tracker.py --interval 2.0"
    "carrier_triple_difference_engine.py --interval 2.0"
    "solar_flare_sid_monitor.py --interval 2.0"
    "multi_frequency_linear_combinations.py --interval 2.0"
    "ppp_sequential_ekf_engine.py --interval 2.0"
    "solar_noon_photochemistry_engine.py --interval 2.0"
    "frontend_iq_imbalance_sounder.py --interval 2.0"
    "lambda_ambiguity_resolution_engine.py --interval 2.0"
    "solar_radiation_pressure_sounder.py --interval 2.0"
    "agw_tid_wavevector_engine.py --interval 2.0"
    "earth_solid_tide_sounder.py --interval 2.0"
    "tropospheric_refractivity_ducting_sounder.py --interval 2.0"
    "satellite_atomic_clock_analyzer.py --interval 2.0"
)

start_all() {
    echo "=== Starting All 30 GNSS Metrology Daemons ==="
    cd "$WORKDIR" || exit 1
    for d in "${DAEMONS[@]}"; do
        script=$(echo "$d" | awk '{print $1}')
        args=$(echo "$d" | cut -d' ' -f2-)
        name=$(basename "$script" .py)
        if pgrep -f "$script" > /dev/null; then
            echo "[RUNNING] $name already active (PID $(pgrep -f "$script" | head -1))"
        else
            nohup python3 "scripts/$script" $args </dev/null >> "$LOGDIR/$name.log" 2>&1 &
            sleep 0.2
            echo "[STARTED] $name (PID $!)"
        fi
    done
}

stop_all() {
    echo "=== Stopping All GNSS Metrology Daemons ==="
    for d in "${DAEMONS[@]}"; do
        script=$(echo "$d" | awk '{print $1}')
        name=$(basename "$script" .py)
        if pgrep -f "$script" > /dev/null; then
            pkill -f "$script"
            echo "[STOPPED] $name"
        else
            echo "[INACTIVE] $name"
        fi
    done
}

status_all() {
    echo "=== GNSS Metrology Suite Health & Status ==="
    printf "%-35s %-10s %-12s %-20s
" "DAEMON NAME" "STATUS" "PID" "LOG FILE"
    printf "%-35s %-10s %-12s %-20s
" "-----------------------------------" "------" "---" "--------"
    local running_count=0
    for d in "${DAEMONS[@]}"; do
        script=$(echo "$d" | awk '{print $1}')
        name=$(basename "$script" .py)
        pid=$(pgrep -f "$script" | head -1)
        if [ -n "$pid" ]; then
            printf "%-35s [0;32m%-10s[0m %-12s %-20s
" "$name" "RUNNING" "$pid" "$LOGDIR/$name.log"
            ((running_count++))
        else
            printf "%-35s [0;31m%-10s[0m %-12s %-20s
" "$name" "DEAD" "--" "--"
        fi
    done
    echo "--------------------------------------------------------------------------------"
    echo "Total Active Metrology Daemons: $running_count / ${#DAEMONS[@]}"
}

case "${1:-status}" in
    start)   start_all ;;
    stop)    stop_all ;;
    restart) stop_all; sleep 1; start_all ;;
    status)  status_all ;;
    *)       echo "Usage: $0 {start|stop|restart|status}"; exit 1 ;;
esac
