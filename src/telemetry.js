import { invokeGemini } from './ai.js';

document.getElementById('csvFileInput').addEventListener('change', function (event) {
    const file = event.target.files[0];
    if (!file) return;

    Papa.parse(file, {
        header: true,
        dynamicTyping: true,
        skipEmptyLines: true,
        complete: function (results) {
            processData(results.data);
        }
    });
});

function getProp(row, exactNames, def = undefined) {
    const keys = Object.keys(row);
    for (let name of exactNames) {
        const nameLower = name.toLowerCase();
        for (let k of keys) {
            if (k.toLowerCase() === nameLower && row[k] !== undefined && row[k] !== null && row[k] !== '') {
                let val = Number(row[k]);
                if (!isNaN(val)) return val;
            }
        }
    }
    return def;
}

function getMinMax(array) {
    if (array.length === 0) return { min: 0, max: 0 };
    let min = Infinity;
    let max = -Infinity;
    for (let i = 0; i < array.length; i++) {
        if (array[i] < min) min = array[i];
        if (array[i] > max) max = array[i];
    }
    return { min, max };
}

function computeDamperVelocity(travels, times) {
    let v = [];
    for (let i = 1; i < travels.length; i++) {
        let dt = times[i] - times[i - 1];
        if (dt <= 0) continue;

        let dx = travels[i] - travels[i - 1];
        let vel = (dx / dt) * 1000000;
        v.push(vel);
    }

    let smoothed = [];
    let window = 2;
    for (let i = 0; i < v.length; i++) {
        let sum = 0;
        let count = 0;
        for (let j = Math.max(0, i - window); j <= Math.min(v.length - 1, i + window); j++) {
            sum += v[j];
            count++;
        }
        smoothed.push(sum / count);
    }
    return smoothed;
}

function asymmetricMovingAverage(arr, window) {
    let res = [];
    for (let i = 0; i < arr.length; i++) {
        let sum = 0; let count = 0;
        for (let j = Math.max(0, i - window); j <= Math.min(arr.length - 1, i + window); j++) {
            sum += arr[j]; count++;
        }
        let local_mean = sum / count;
        res.push(Math.max(arr[i], local_mean));
    }
    return res;
}

function calculateOptimalTempRange(pairs) {
    if (pairs.length < 10) return { min: 190, max: 220 };

    pairs.sort((a, b) => a.slip - b.slip);

    let eliteCount = Math.floor(pairs.length * 0.05);
    if (eliteCount < 5) eliteCount = Math.min(5, pairs.length);

    let eliteTemps = [];
    for (let i = 0; i < eliteCount; i++) {
        eliteTemps.push(pairs[i].temp);
    }

    eliteTemps.sort((a, b) => a - b);

    function getPercentile(arr, p) {
        let index = (arr.length - 1) * p;
        let lower = Math.floor(index);
        let upper = Math.ceil(index);
        let weight = index - lower;
        if (upper >= arr.length) return arr[lower];
        return arr[lower] * (1 - weight) + arr[upper] * weight;
    }

    let minT = getPercentile(eliteTemps, 0.25);
    let maxT = getPercentile(eliteTemps, 0.75);

    if (minT === maxT) { minT -= 2; maxT += 2; }

    return { min: minT, max: maxT };
}

function processData(rawData) {
    const data = rawData;

    const times = [];
    const track_x = []; const track_z = []; const track_state = [];

    const susp_m_fl = []; const susp_m_fr = []; const susp_m_rl = []; const susp_m_rr = [];

    const susp_norm_fl_corner = []; const susp_norm_fr_corner = [];
    const susp_norm_rl_corner = []; const susp_norm_rr_corner = [];

    const susp_norm_fl_straight = []; const susp_norm_fr_straight = [];
    const susp_norm_rl_straight = []; const susp_norm_rr_straight = [];

    const temp_fl_corner = []; const temp_fr_corner = []; const temp_rl_corner = []; const temp_rr_corner = [];
    const temp_fl_straight = []; const temp_fr_straight = []; const temp_rl_straight = []; const temp_rr_straight = [];
    const temp_slip_pairs_front = []; const temp_slip_pairs_rear = [];

    const diff_f_accel = []; const diff_r_accel = []; const diff_c_accel = [];
    const diff_f_decel = []; const diff_r_decel = []; const diff_c_decel = [];

    const engine_rpm = []; const engine_power = []; const engine_torque = [];
    const engine_accel = []; const engine_steer = []; const engine_brake = [];
    const engine_time = []; const engine_speed = [];
    let max_accel_recorded = 0;

    let drivetrain_val = undefined;
    let current_corner_state = 0;

    data.forEach(row => {
        if (drivetrain_val === undefined) {
            let val = getProp(row, ['drivetraintype', 'drivetrain']);
            if (val !== undefined) drivetrain_val = val;
        }

        let t = getProp(row, ['timestampms', 't_ms', 'time', 'timestamp']);
        let speed = getProp(row, ['speed', 'velocity', 'speed_kmh', 'speed_ms', 'speed_mph'], 0);
        let yaw = Math.abs(getProp(row, ['angularvelocityy', 'yawrate', 'yaw_rate', 'yaw_velocity'], 0));

        let raw_steer = getProp(row, ['steer', 'steering', 'steer_angle'], 0);
        let steer_norm = Math.abs(raw_steer) > 1.0 ? raw_steer / 127.0 : raw_steer;
        let steer = Math.abs(steer_norm);

        let px = getProp(row, ['positionx', 'posx', 'pos_x', 'x']);
        let pz = getProp(row, ['positionz', 'posz', 'pos_z', 'z', 'positiony', 'posy', 'pos_y', 'y']);

        let s_m_fl = getProp(row, ['suspensiontravelmetersfrontleft', 'susp_fl_m', 'suspension_fl']);
        let s_m_fr = getProp(row, ['suspensiontravelmetersfrontright', 'susp_fr_m', 'suspension_fr']);
        let s_m_rl = getProp(row, ['suspensiontravelmetersrearleft', 'susp_rl_m', 'suspension_rl']);
        let s_m_rr = getProp(row, ['suspensiontravelmetersrearright', 'susp_rr_m', 'suspension_rr']);

        let s_n_fl = getProp(row, ['normalizedsuspensiontravelfrontleft', 'susp_norm_fl']);
        let s_n_fr = getProp(row, ['normalizedsuspensiontravelfrontright', 'susp_norm_fr']);
        let s_n_rl = getProp(row, ['normalizedsuspensiontravelrearleft', 'susp_norm_rl']);
        let s_n_rr = getProp(row, ['normalizedsuspensiontravelrearright', 'susp_norm_rr']);

        if (s_n_fl === undefined) s_n_fl = s_m_fl;
        if (s_n_fr === undefined) s_n_fr = s_m_fr;
        if (s_n_rl === undefined) s_n_rl = s_m_rl;
        if (s_n_rr === undefined) s_n_rr = s_m_rr;

        if (current_corner_state === 0) {
            if (yaw >= 0.50) current_corner_state = 1;
        } else {
            if (yaw < 0.30) current_corner_state = 0;
        }

        let is_cornering = current_corner_state === 1;
        let is_straight = current_corner_state === 0;

        let is_airborne = false;

        if (px !== undefined && pz !== undefined && t !== undefined) {
            track_x.push(px);
            track_z.push(pz);
            track_state.push(current_corner_state);
        }

        if (t !== undefined && s_m_fl !== undefined) {
            times.push(t);
            susp_m_fl.push(s_m_fl); susp_m_fr.push(s_m_fr);
            susp_m_rl.push(s_m_rl); susp_m_rr.push(s_m_rr);

            if (s_n_fl !== undefined) {
                if (is_cornering) {
                    susp_norm_fl_corner.push(s_n_fl); susp_norm_fr_corner.push(s_n_fr);
                    susp_norm_rl_corner.push(s_n_rl); susp_norm_rr_corner.push(s_n_rr);
                } else if (is_straight) {
                    susp_norm_fl_straight.push(s_n_fl); susp_norm_fr_straight.push(s_n_fr);
                    susp_norm_rl_straight.push(s_n_rl); susp_norm_rr_straight.push(s_n_rr);
                }
            }
        }

        let t_fl = getProp(row, ['tiretempfrontleft', 'tiretempfl', 'temp_fl', 'tire_temp_fl']);
        let t_fr = getProp(row, ['tiretempfrontright', 'tiretempfr', 'temp_fr', 'tire_temp_fr']);
        let t_rl = getProp(row, ['tiretemprearleft', 'tiretemprl', 'temp_rl', 'tire_temp_rl']);
        let t_rr = getProp(row, ['tiretemprearright', 'tiretemprr', 'temp_rr', 'tire_temp_rr']);

        let raw_accel = getProp(row, ['accelerator', 'accel', 'throttle', 'throttle_pct', 'throttle_input'], 0);
        let accel = raw_accel > 1.0 ? raw_accel / 255.0 : raw_accel;

        let raw_brake = getProp(row, ['brake', 'decel', 'brake_pct', 'brake_input'], 0);
        let brake = raw_brake > 1.0 ? raw_brake / 255.0 : raw_brake;

        let is_high_load = is_cornering || accel > 0.5 || brake > 0.5;

        if (t_fl !== undefined && !is_airborne) {
            if (is_cornering) {
                temp_fl_corner.push(t_fl); temp_fr_corner.push(t_fr);
                temp_rl_corner.push(t_rl); temp_rr_corner.push(t_rr);
            } else if (is_straight) {
                temp_fl_straight.push(t_fl); temp_fr_straight.push(t_fr);
                temp_rl_straight.push(t_rl); temp_rr_straight.push(t_rr);
            }
        }

        if (!is_airborne && is_high_load) {
            let cs_fl = getProp(row, ['tirecombinedslipfrontleft', 'combined_slip_fl']);
            let cs_fr = getProp(row, ['tirecombinedslipfrontright', 'combined_slip_fr']);
            let cs_rl = getProp(row, ['tirecombinedsliprearleft', 'combined_slip_rl']);
            let cs_rr = getProp(row, ['tirecombinedsliprearright', 'combined_slip_rr']);

            if (t_fl !== undefined && cs_fl !== undefined && Math.abs(cs_fl) <= 1.0) temp_slip_pairs_front.push({ temp: t_fl, slip: Math.abs(cs_fl) });
            if (t_fr !== undefined && cs_fr !== undefined && Math.abs(cs_fr) <= 1.0) temp_slip_pairs_front.push({ temp: t_fr, slip: Math.abs(cs_fr) });
            if (t_rl !== undefined && cs_rl !== undefined && Math.abs(cs_rl) <= 1.0) temp_slip_pairs_rear.push({ temp: t_rl, slip: Math.abs(cs_rl) });
            if (t_rr !== undefined && cs_rr !== undefined && Math.abs(cs_rr) <= 1.0) temp_slip_pairs_rear.push({ temp: t_rr, slip: Math.abs(cs_rr) });
        }

        let w_fl = getProp(row, ['wheelrotationspeedfrontleft', 'wheel_speed_fl', 'speed_fl', 'w_fl']);
        let w_fr = getProp(row, ['wheelrotationspeedfrontright', 'wheel_speed_fr', 'speed_fr', 'w_fr']);
        let w_rl = getProp(row, ['wheelrotationspeedrearleft', 'wheel_speed_rl', 'speed_rl', 'w_rl']);
        let w_rr = getProp(row, ['wheelrotationspeedrearright', 'wheel_speed_rr', 'speed_rr', 'w_rr']);

        if (yaw > 0 && w_fl !== undefined && w_fr !== undefined && w_rl !== undefined && w_rr !== undefined) {
            let is_exit = accel > 0 && accel > brake;
            let is_entry = (brake > 0 && brake >= accel) || (accel === 0 && brake === 0);

            let sum_f = w_fl + w_fr;
            let sum_r = w_rl + w_rr;

            let delta_f = sum_f !== 0 ? Math.abs((w_fl - w_fr) / sum_f) : 0;
            let delta_r = sum_r !== 0 ? Math.abs((w_rl - w_rr) / sum_r) : 0;

            let avg_f = sum_f / 2;
            let avg_r = sum_r / 2;
            let sum_c = avg_f + avg_r;

            let delta_c = sum_c !== 0 ? (avg_f - avg_r) / sum_c : 0;

            if (is_exit) {
                diff_f_accel.push(delta_f);
                diff_r_accel.push(delta_r);
                diff_c_accel.push(delta_c);
            } else if (is_entry) {
                diff_f_decel.push(delta_f);
                diff_r_decel.push(delta_r);
                diff_c_decel.push(delta_c);
            }
        }

        let engine_r = getProp(row, ['currentenginerpm', 'enginerpm', 'rpm']);
        let power_val = getProp(row, ['power', 'power_w']);
        let torque_val = getProp(row, ['torque', 'torque_nm']);

        if (accel > max_accel_recorded) max_accel_recorded = accel;

        if (engine_r !== undefined && power_val !== undefined && torque_val !== undefined && t !== undefined) {
            engine_rpm.push(engine_r);
            engine_power.push(power_val);
            engine_torque.push(torque_val);
            engine_accel.push(accel);
            engine_steer.push(steer);
            engine_brake.push(brake);
            engine_time.push(t);
            engine_speed.push(speed);
        }
    });

    document.getElementById('trackMapSamples').innerText = track_x.length.toLocaleString() + ' GPS frames';
    if (track_x.length > 0) {
        plotTrackMap(track_x, track_z, track_state, 'trackMapPlot');
    }

    const optRangeFront = calculateOptimalTempRange(temp_slip_pairs_front);
    const optRangeRear = calculateOptimalTempRange(temp_slip_pairs_rear);

    const wot_threshold = max_accel_recorded * 0.95;
    const wot_data = [];

    const MAX_RPM_ACCEL = 10000;
    const MAX_POS_TQ_DERIV = 15000;
    const MAX_NEG_TQ_DERIV = -2000;
    const SETTLING_DELAY_MS = 125;

    let prev_t = null; let prev_rpm = null; let prev_speed = null; let prev_tq = null;
    let sweep_start_time = null;

    for (let i = 0; i < engine_rpm.length; i++) {
        if (engine_accel[i] >= wot_threshold && engine_steer[i] < 0.05 && engine_brake[i] === 0) {
            let t = engine_time[i]; let rpm = engine_rpm[i]; let speed = engine_speed[i];
            let tq = engine_torque[i] * 0.73756; let hp = engine_power[i] / 745.7;

            if (sweep_start_time === null) sweep_start_time = t;

            if (prev_t !== null && t > prev_t) {
                let dt = t - prev_t; let dt_sec = dt > 1 ? dt / 1000.0 : dt;

                if (dt_sec > 0 && dt_sec < 0.5) {
                    let dRPM_dt = (rpm - prev_rpm) / dt_sec;
                    let dSpeed_dt = (speed - prev_speed) / dt_sec;
                    let dTorque_dt = (tq - prev_tq) / dt_sec;

                    let is_monotonic = dRPM_dt > 0;
                    let is_torque_stable = dTorque_dt < MAX_POS_TQ_DERIV && dTorque_dt > MAX_NEG_TQ_DERIV;
                    let is_coupled = dSpeed_dt > -0.5 && dRPM_dt < MAX_RPM_ACCEL;
                    let is_settled = (t - sweep_start_time) >= SETTLING_DELAY_MS;

                    if (is_monotonic && is_torque_stable && is_coupled && is_settled) {
                        wot_data.push({ r: rpm, hp: hp, tq: tq });
                    }
                }
            }
            prev_t = t; prev_rpm = rpm; prev_speed = speed; prev_tq = tq;
        } else {
            prev_t = null; sweep_start_time = null;
        }
    }

    wot_data.sort((a, b) => a.r - b.r);

    const filtered_wot_data = [];
    const stat_window = 10;

    for (let i = 0; i < wot_data.length; i++) {
        let sum = 0; let count = 0;
        let start = Math.max(0, i - stat_window);
        let end = Math.min(wot_data.length - 1, i + stat_window);

        for (let j = start; j <= end; j++) {
            sum += wot_data[j].hp; count++;
        }
        let mean = sum / count;

        let variance_sum = 0;
        for (let j = start; j <= end; j++) {
            variance_sum += Math.pow(wot_data[j].hp - mean, 2);
        }
        let std_dev = Math.sqrt(variance_sum / count);
        if (std_dev < 0.1) std_dev = 0.1;

        let z_score = (wot_data[i].hp - mean) / std_dev;
        if (z_score <= 2) filtered_wot_data.push(wot_data[i]);
    }

    const rpm_groups = {};

    for (let i = 0; i < filtered_wot_data.length; i++) {
        let r = Math.round(filtered_wot_data[i].r);
        let hp = filtered_wot_data[i].hp;
        let tq = filtered_wot_data[i].tq;

        if (!rpm_groups[r]) {
            rpm_groups[r] = { hp: hp, tq: tq };
        } else {
            if (hp > rpm_groups[r].hp) {
                rpm_groups[r].hp = hp; rpm_groups[r].tq = tq;
            }
        }
    }

    const sorted_rpms = Object.keys(rpm_groups).map(Number).sort((a, b) => a - b);
    const raw_hp = []; const raw_tq = []; const plot_rpm = [];

    for (let i = 0; i < sorted_rpms.length; i++) {
        plot_rpm.push(sorted_rpms[i]);
        raw_hp.push(rpm_groups[sorted_rpms[i]].hp);
        raw_tq.push(rpm_groups[sorted_rpms[i]].tq);
    }

    const window_size = 3;
    const plot_hp = asymmetricMovingAverage(raw_hp, window_size);
    const plot_tq = asymmetricMovingAverage(raw_tq, window_size);

    document.getElementById('dynoSamples').innerText = filtered_wot_data.length.toLocaleString() + ' valid frames';
    let max_extracted_hp = 0; let max_extracted_tq = 0;
    if (plot_rpm.length > 0) {
        const maxVals = plotDynoScatter(plot_rpm, plot_hp, plot_tq, 'dynoPlot');
        max_extracted_hp = maxVals.hp; max_extracted_tq = maxVals.tq;
    }

    const front_temp_corner = temp_fl_corner.concat(temp_fr_corner);
    const rear_temp_corner = temp_rl_corner.concat(temp_rr_corner);
    const front_temp_straight = temp_fl_straight.concat(temp_fr_straight);
    const rear_temp_straight = temp_rl_straight.concat(temp_rr_straight);

    document.getElementById('frontTempCorneringSamples').innerText = front_temp_corner.length.toLocaleString() + ' samples';
    document.getElementById('rearTempCorneringSamples').innerText = rear_temp_corner.length.toLocaleString() + ' samples';
    document.getElementById('frontTempStraightSamples').innerText = front_temp_straight.length.toLocaleString() + ' samples';
    document.getElementById('rearTempStraightSamples').innerText = rear_temp_straight.length.toLocaleString() + ' samples';

    let frontTempCornerStats = { inZonePct: 0, mean: 0, p01: 0, p50: 0, p99: 0, mad: 0 };
    let rearTempCornerStats = { inZonePct: 0, mean: 0, p01: 0, p50: 0, p99: 0, mad: 0 };
    let frontTempStraightStats = { inZonePct: 0, mean: 0, p01: 0, p50: 0, p99: 0, mad: 0 };
    let rearTempStraightStats = { inZonePct: 0, mean: 0, p01: 0, p50: 0, p99: 0, mad: 0 };

    if (front_temp_corner.length > 0) {
        const ext = getMinMax(front_temp_corner);
        frontTempCornerStats = plotTempHistogram(front_temp_corner, 'frontTempCorneringPlot', ext.min, ext.max, 20, '#2979ff', optRangeFront.min, optRangeFront.max);
    }
    if (rear_temp_corner.length > 0) {
        const ext = getMinMax(rear_temp_corner);
        rearTempCornerStats = plotTempHistogram(rear_temp_corner, 'rearTempCorneringPlot', ext.min, ext.max, 20, '#2979ff', optRangeRear.min, optRangeRear.max);
    }
    if (front_temp_straight.length > 0) {
        const ext = getMinMax(front_temp_straight);
        frontTempStraightStats = plotTempHistogram(front_temp_straight, 'frontTempStraightPlot', ext.min, ext.max, 20, '#d95829', optRangeFront.min, optRangeFront.max);
    }
    if (rear_temp_straight.length > 0) {
        const ext = getMinMax(rear_temp_straight);
        rearTempStraightStats = plotTempHistogram(rear_temp_straight, 'rearTempStraightPlot', ext.min, ext.max, 20, '#d95829', optRangeRear.min, optRangeRear.max);
    }

    const front_susp_corner = susp_norm_fl_corner.concat(susp_norm_fr_corner);
    const rear_susp_corner = susp_norm_rl_corner.concat(susp_norm_rr_corner);
    const front_susp_straight = susp_norm_fl_straight.concat(susp_norm_fr_straight);
    const rear_susp_straight = susp_norm_rl_straight.concat(susp_norm_rr_straight);

    document.getElementById('frontCorneringSamples').innerText = front_susp_corner.length.toLocaleString() + ' samples';
    document.getElementById('rearCorneringSamples').innerText = rear_susp_corner.length.toLocaleString() + ' samples';
    document.getElementById('frontStraightSamples').innerText = front_susp_straight.length.toLocaleString() + ' samples';
    document.getElementById('rearStraightSamples').innerText = rear_susp_straight.length.toLocaleString() + ' samples';

    let frontCorneringStats = { p01: 0, p50: 0, p99: 0, mad: 0 };
    let rearCorneringStats = { p01: 0, p50: 0, p99: 0, mad: 0 };
    let frontStraightStats = { p01: 0, p50: 0, p99: 0, mad: 0 };
    let rearStraightStats = { p01: 0, p50: 0, p99: 0, mad: 0 };

    if (front_susp_corner.length > 0) {
        frontCorneringStats = plotSuspensionHistogram(front_susp_corner, 'frontCorneringPlot', 0, 1, 20, '#2979ff');
    }
    if (rear_susp_corner.length > 0) {
        rearCorneringStats = plotSuspensionHistogram(rear_susp_corner, 'rearCorneringPlot', 0, 1, 20, '#2979ff');
    }
    if (front_susp_straight.length > 0) {
        frontStraightStats = plotSuspensionHistogram(front_susp_straight, 'frontStraightPlot', 0, 1, 20, '#d95829');
    }
    if (rear_susp_straight.length > 0) {
        rearStraightStats = plotSuspensionHistogram(rear_susp_straight, 'rearStraightPlot', 0, 1, 20, '#d95829');
    }

    const v_fl = computeDamperVelocity(susp_m_fl, times);
    const v_fr = computeDamperVelocity(susp_m_fr, times);
    const v_rl = computeDamperVelocity(susp_m_rl, times);
    const v_rr = computeDamperVelocity(susp_m_rr, times);
    const front_vel_combined = v_fl.concat(v_fr);
    const rear_vel_combined = v_rl.concat(v_rr);
    document.getElementById('frontDamperSamples').innerText = front_vel_combined.length.toLocaleString() + ' samples';
    document.getElementById('rearDamperSamples').innerText = rear_vel_combined.length.toLocaleString() + ' samples';
    if (front_vel_combined.length > 0) plotDamperHistogram(front_vel_combined, 'frontDamperPlot');
    if (rear_vel_combined.length > 0) plotDamperHistogram(rear_vel_combined, 'rearDamperPlot');

    let diff_f_accel_stats = { p95: 0, p99: 0 }, diff_r_accel_stats = { p95: 0, p99: 0 }, mean_accel_c = { mean: 0 };
    let diff_f_decel_stats = { p95: 0, p99: 0 }, diff_r_decel_stats = { p95: 0, p99: 0 }, mean_decel_c = { mean: 0 };

    document.getElementById('accelFrontDiffSamples').innerText = diff_f_accel.length.toLocaleString() + ' samples';
    if (diff_f_accel.length > 0) {
        let ext = getMinMax(diff_f_accel);
        diff_f_accel_stats = plotHistogram(diff_f_accel, 'accelFrontDiffPlot', ext.min, ext.max, 20, '#d95829', false, false);
    }

    document.getElementById('accelRearDiffSamples').innerText = diff_r_accel.length.toLocaleString() + ' samples';
    if (diff_r_accel.length > 0) {
        let ext = getMinMax(diff_r_accel);
        diff_r_accel_stats = plotHistogram(diff_r_accel, 'accelRearDiffPlot', ext.min, ext.max, 20, '#d95829', false, false);
    }

    document.getElementById('accelCenterDiffSamples').innerText = diff_c_accel.length.toLocaleString() + ' samples';
    if (diff_c_accel.length > 0) {
        let ext = getMinMax(diff_c_accel);
        mean_accel_c = plotHistogram(diff_c_accel, 'accelCenterDiffPlot', ext.min, ext.max, 20, '#d95829', true, true);
    }

    document.getElementById('decelFrontDiffSamples').innerText = diff_f_decel.length.toLocaleString() + ' samples';
    if (diff_f_decel.length > 0) {
        let ext = getMinMax(diff_f_decel);
        diff_f_decel_stats = plotHistogram(diff_f_decel, 'decelFrontDiffPlot', ext.min, ext.max, 20, '#2979ff', false, false);
    }

    document.getElementById('decelRearDiffSamples').innerText = diff_r_decel.length.toLocaleString() + ' samples';
    if (diff_r_decel.length > 0) {
        let ext = getMinMax(diff_r_decel);
        diff_r_decel_stats = plotHistogram(diff_r_decel, 'decelRearDiffPlot', ext.min, ext.max, 20, '#2979ff', false, false);
    }

    document.getElementById('decelCenterDiffSamples').innerText = diff_c_decel.length.toLocaleString() + ' samples';
    if (diff_c_decel.length > 0) {
        let ext = getMinMax(diff_c_decel);
        mean_decel_c = plotHistogram(diff_c_decel, 'decelCenterDiffPlot', ext.min, ext.max, 20, '#2979ff', true, true);
    }

    let drivetrain_str = "UNKNOWN";
    if (drivetrain_val === 1) drivetrain_str = "RWD";
    else if (drivetrain_val === 0) drivetrain_str = "FWD";
    else if (drivetrain_val === 2) drivetrain_str = "AWD";

    document.getElementById('drivetrainLabel').innerText = "DRIVETRAIN: " + drivetrain_str;

    const payload = {
        drivetrain: drivetrain_str,
        engine: { max_hp: max_extracted_hp, max_tq: max_extracted_tq },
        optimal_temp_front: optRangeFront,
        optimal_temp_rear: optRangeRear,
        temp_stats: {
            cornering_front: frontTempCornerStats,
            cornering_rear: rearTempCornerStats,
            straight_front: frontTempStraightStats,
            straight_rear: rearTempStraightStats
        },
        susp_stats: {
            cornering_front: frontCorneringStats,
            cornering_rear: rearCorneringStats,
            straight_front: frontStraightStats,
            straight_rear: rearStraightStats
        },
        diff_stats: { accel_f: diff_f_accel_stats, accel_r: diff_r_accel_stats, decel_f: diff_f_decel_stats, decel_r: diff_r_decel_stats },
        diff_mean: { accel_c: mean_accel_c.mean || 0, decel_c: mean_decel_c.mean || 0 }
    };

    setTimeout(() => invokeGemini(payload), 500);
}

function plotTrackMap(x_data, z_data, states, elementId) {
    let straight_x = [], straight_z = [];
    let corner_x = [], corner_z = [];

    for (let i = 0; i < x_data.length; i++) {
        if (states[i] === 0) {
            straight_x.push(x_data[i]);
            straight_z.push(z_data[i]);
        } else {
            corner_x.push(x_data[i]);
            corner_z.push(z_data[i]);
        }
    }

    const traceStraight = {
        x: straight_x, y: straight_z,
        mode: 'markers', type: 'scattergl', name: 'STRAIGHT',
        marker: { color: '#d95829', size: 2 }
    };

    const traceCorner = {
        x: corner_x, y: corner_z,
        mode: 'markers', type: 'scattergl', name: 'CORNER',
        marker: { color: '#2979ff', size: 2 }
    };

    const layout = {
        paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', showlegend: false,
        font: { family: '"JetBrains Mono", monospace', color: '#62646c', size: 10 },
        xaxis: { showgrid: false, zeroline: false, showticklabels: false, fixedrange: true },
        yaxis: { showgrid: false, zeroline: false, showticklabels: false, scaleanchor: 'x', scaleratio: 1, fixedrange: true },
        margin: { t: 10, b: 10, l: 10, r: 10 }, hovermode: false
    };

    Plotly.newPlot(elementId, [traceStraight, traceCorner], layout, { displayModeBar: false });
}

function plotTempHistogram(data, elementId, min, max, numBins, color, optMin, optMax) {
    const size = max === min ? 1 : (max - min) / numBins;
    const trace = { x: data, type: 'histogram', autobinx: false, xbins: { start: min, end: max, size: size }, marker: { color: color }, opacity: 1 };

    let inZoneCount = 0;
    let sum = 0;
    for (let i = 0; i < data.length; i++) {
        if (data[i] >= optMin && data[i] <= optMax) inZoneCount++;
        sum += data[i];
    }
    let inZonePct = data.length > 0 ? (inZoneCount / data.length * 100).toFixed(1) : 0;
    let mean = data.length > 0 ? sum / data.length : 0;

    let p01 = 0, p50 = 0, p99 = 0, mad = 0;

    if (data.length > 0) {
        const sorted = [...data].sort((a, b) => a - b);
        const getP = (arr, p) => {
            const idx = (arr.length - 1) * p;
            const base = Math.floor(idx);
            const rest = idx - base;
            return arr[base + 1] !== undefined ? arr[base] + rest * (arr[base + 1] - arr[base]) : arr[base];
        };

        p01 = getP(sorted, 0.01);
        p50 = getP(sorted, 0.50);
        p99 = getP(sorted, 0.99);

        const deviations = data.map(x => Math.abs(x - p50)).sort((a, b) => a - b);
        mad = getP(deviations, 0.50);
    }

    const layout = {
        paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)',
        font: { family: '"JetBrains Mono", monospace', color: '#62646c', size: 10 },
        xaxis: { showgrid: true, gridcolor: '#23252a', zeroline: false, fixedrange: true },
        yaxis: { showgrid: false, zeroline: false, showticklabels: false, fixedrange: true },
        margin: { t: 0, b: 30, l: 0, r: 0 }, bargap: 0.15,
        shapes: [
            {
                type: 'rect', xref: 'x', yref: 'paper',
                x0: optMin, x1: optMax, y0: 0, y1: 1,
                fillcolor: 'rgba(217, 88, 41, 0.15)', line: { width: 0 }, layer: 'below'
            },
            {
                type: 'line', xref: 'x', yref: 'paper',
                x0: mean, x1: mean, y0: 0, y1: 1,
                line: { color: '#d1d2d5', width: 2, dash: 'dot' }, layer: 'above'
            }
        ],
        annotations: [{
            text: `TARGET: ${optMin.toFixed(0)}° - ${optMax.toFixed(0)}° | IN-ZONE: ${inZonePct}%<br>P01: ${p01.toFixed(1)}° | P50: ${p50.toFixed(1)}°<br>P99: ${p99.toFixed(1)}° | MAD: ${mad.toFixed(2)}°`,
            xref: 'paper', yref: 'paper', x: 1, y: 1, showarrow: false,
            font: { family: '"JetBrains Mono", monospace', size: 10, color: '#d1d2d5' },
            align: 'right', bgcolor: '#121318', bordercolor: '#23252a', borderpad: 4
        }]
    };
    Plotly.newPlot(elementId, [trace], layout, { displayModeBar: false });

    return { inZonePct: inZonePct, mean: mean, p01: p01, p50: p50, p99: p99, mad: mad };
}

function plotSuspensionHistogram(data, elementId, min, max, numBins, color) {
    const size = max === min ? 1 : (max - min) / numBins;

    const trace = {
        x: data, type: 'histogram', autobinx: false, xbins: { start: min, end: max, size: size },
        marker: { color: color }, opacity: 0.75
    };

    let p01 = 0, p50 = 0, p99 = 0, mad = 0;

    if (data.length > 0) {
        const sorted = [...data].sort((a, b) => a - b);
        const getP = (arr, p) => {
            const idx = (arr.length - 1) * p;
            const base = Math.floor(idx);
            const rest = idx - base;
            return arr[base + 1] !== undefined ? arr[base] + rest * (arr[base + 1] - arr[base]) : arr[base];
        };

        p01 = getP(sorted, 0.01);
        p50 = getP(sorted, 0.50);
        p99 = getP(sorted, 0.99);

        const deviations = data.map(x => Math.abs(x - p50)).sort((a, b) => a - b);
        mad = getP(deviations, 0.50);
    }

    const layout = {
        paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', showlegend: false,
        font: { family: '"JetBrains Mono", monospace', color: '#62646c', size: 10 },
        xaxis: { showgrid: true, gridcolor: '#23252a', zeroline: false, fixedrange: true },
        yaxis: { showgrid: false, zeroline: false, showticklabels: false, fixedrange: true },
        margin: { t: 0, b: 30, l: 0, r: 0 }, bargap: 0.15,
        annotations: [{
            text: `P01: ${p01.toFixed(3)} | P50: ${p50.toFixed(3)}<br>P99: ${p99.toFixed(3)} | MAD: ${mad.toFixed(3)}`,
            xref: 'paper', yref: 'paper', x: 1, y: 1, showarrow: false,
            font: { family: '"JetBrains Mono", monospace', size: 10, color: '#d1d2d5' },
            align: 'right', bgcolor: '#121318', bordercolor: '#23252a', borderpad: 4
        }]
    };

    Plotly.newPlot(elementId, [trace], layout, { displayModeBar: false });

    return { p01: p01, p50: p50, p99: p99, mad: mad };
}

function plotDynoScatter(rpm, hp, tq, elementId) {
    let maxHP = 0; let maxHPRPM = 0; let maxTQ = 0; let maxTQRPM = 0;
    for (let i = 0; i < hp.length; i++) {
        if (hp[i] > maxHP) { maxHP = hp[i]; maxHPRPM = rpm[i]; }
        if (tq[i] > maxTQ) { maxTQ = tq[i]; maxTQRPM = rpm[i]; }
    }

    const traceHP = { x: rpm, y: hp, mode: 'lines', type: 'scatter', name: `MAX POWER = ${Math.round(maxHP)} AT ENGINE RPM = ${Math.round(maxHPRPM)}`, line: { color: '#d95829', width: 3, shape: 'spline' } };
    const traceTQ = { x: rpm, y: tq, mode: 'lines', type: 'scatter', name: `MAX TORQUE = ${Math.round(maxTQ)} AT ENGINE RPM = ${Math.round(maxTQRPM)}`, line: { color: '#2979ff', width: 3, shape: 'spline' } };

    const layout = {
        paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', showlegend: true,
        legend: { orientation: 'h', yanchor: 'bottom', y: 1.02, xanchor: 'center', x: 0.5, font: { family: '"JetBrains Mono", monospace', size: 10, color: '#d1d2d5' } },
        font: { family: '"JetBrains Mono", monospace', color: '#62646c', size: 10 },
        xaxis: { showgrid: true, gridcolor: '#23252a', zeroline: false, title: { text: 'RPM', font: { size: 10, color: '#62646c' } } },
        yaxis: { showgrid: true, gridcolor: '#23252a', zeroline: false, title: { text: 'HP / LB-FT', font: { size: 10, color: '#62646c' } } },
        margin: { t: 30, b: 40, l: 40, r: 10 }, hovermode: 'x unified'
    };

    Plotly.newPlot(elementId, [traceHP, traceTQ], layout, { displayModeBar: false });
    return { hp: maxHP, tq: maxTQ };
}

function plotDamperHistogram(velocities, elementId) {
    let max_abs = 0;
    velocities.forEach(v => {
        let abs_v = Math.abs(v);
        if (abs_v > max_abs) max_abs = abs_v;
    });
    if (max_abs === 0) max_abs = 100;

    const num_bins = 40;
    const bin_width = (max_abs * 2) / num_bins;

    const traceBump = {
        x: velocities.filter(v => v >= 0),
        type: 'histogram', name: 'Bump', marker: { color: '#d95829' },
        xbins: { start: 0, end: max_abs, size: bin_width }
    };

    const traceReb = {
        x: velocities.filter(v => v < 0),
        type: 'histogram', name: 'Rebound', marker: { color: '#2979ff' },
        xbins: { start: -max_abs, end: 0, size: bin_width }
    };

    const threshold = 30;
    const total = velocities.length;
    let lsbCount = 0, hsbCount = 0, lsrCount = 0, hsrCount = 0;

    for (let i = 0; i < total; i++) {
        let v = velocities[i];
        if (v >= 0 && v <= threshold) lsbCount++;
        else if (v > threshold) hsbCount++;
        else if (v >= -threshold && v < 0) lsrCount++;
        else if (v < -threshold) hsrCount++;
    }

    let lsbPct = total > 0 ? (lsbCount / total * 100).toFixed(1) : 0;
    let hsbPct = total > 0 ? (hsbCount / total * 100).toFixed(1) : 0;
    let lsrPct = total > 0 ? (lsrCount / total * 100).toFixed(1) : 0;
    let hsrPct = total > 0 ? (hsrCount / total * 100).toFixed(1) : 0;

    const layout = {
        paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)', barmode: 'overlay', showlegend: false,
        font: { family: '"JetBrains Mono", monospace', color: '#62646c', size: 10 },
        xaxis: { showgrid: true, gridcolor: '#23252a', zeroline: true, zerolinecolor: '#8b8d96', zerolinewidth: 2, title: { text: 'VELOCITY (mm/s)', font: { size: 10, color: '#62646c' } } },
        yaxis: { showgrid: false, zeroline: false, showticklabels: false, fixedrange: true },
        margin: { t: 10, b: 40, l: 10, r: 10 }, bargap: 0.1,
        annotations: [{
            text: `LSB(≤30): ${lsbPct}% | LSR(≥-30): ${lsrPct}%<br>HSB(>30): ${hsbPct}% | HSR(<-30): ${hsrPct}%`,
            xref: 'paper', yref: 'paper', x: 1, y: 1, showarrow: false,
            font: { family: '"JetBrains Mono", monospace', size: 10, color: '#d1d2d5' },
            align: 'right', bgcolor: '#121318', bordercolor: '#23252a', borderpad: 4
        }]
    };

    Plotly.newPlot(elementId, [traceBump, traceReb], layout, { displayModeBar: false });
}

function plotHistogram(data, elementId, min, max, numBins, color, crossZero, isCenter = false) {
    const size = max === min ? 1 : (max - min) / numBins;
    const trace = { x: data, type: 'histogram', autobinx: false, xbins: { start: min, end: max, size: size }, marker: { color: color }, opacity: 1 };

    let annotationText = "";
    let retValue = { p95: 0, p99: 0, mean: 0 };

    if (isCenter) {
        let sum = 0;
        for (let i = 0; i < data.length; i++) sum += data[i];
        retValue.mean = data.length > 0 ? (sum / data.length) : 0;
        annotationText = `MEAN (μ): ${retValue.mean.toFixed(3)}`;
    } else {
        if (data.length > 0) {
            const sorted = [...data].map(Math.abs).sort((a, b) => a - b);
            const getP = (p) => {
                const idx = (sorted.length - 1) * p;
                const base = Math.floor(idx);
                const rest = idx - base;
                return sorted[base + 1] !== undefined ? sorted[base] + rest * (sorted[base + 1] - sorted[base]) : sorted[base];
            };
            retValue.p95 = getP(0.95);
            retValue.p99 = getP(0.99);
        }
        annotationText = `P95: ${retValue.p95.toFixed(3)} | P99: ${retValue.p99.toFixed(3)}`;
    }

    const layout = {
        paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)',
        font: { family: '"JetBrains Mono", monospace', color: '#62646c', size: 10 },
        xaxis: { showgrid: true, gridcolor: '#23252a', zeroline: crossZero, zerolinecolor: '#8b8d96', zerolinewidth: 2, fixedrange: true },
        yaxis: { showgrid: false, zeroline: false, showticklabels: false, fixedrange: true },
        margin: { t: 0, b: 30, l: 0, r: 0 }, bargap: 0.15,
        annotations: [{
            text: annotationText,
            xref: 'paper', yref: 'paper', x: 1, y: 1, showarrow: false,
            font: { family: '"JetBrains Mono", monospace', size: 10, color: '#d1d2d5' },
            align: 'right', bgcolor: '#121318', bordercolor: '#23252a', borderpad: 4
        }]
    };

    Plotly.newPlot(elementId, [trace], layout, { displayModeBar: false });
    return retValue;
}
