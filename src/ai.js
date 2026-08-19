import { GoogleGenAI } from '@google/genai';

export async function invokeGemini(telemetryData) {
    const apiKey = document.getElementById('geminiApiKey').value.trim();
    const outContainer = document.getElementById('geminiOutput');

    if (!apiKey) {
        outContainer.innerHTML = '<span class="status-indicator">INPUT API KEY TO GENERATE RECOMMENDATIONS.</span>';
        return;
    }

    outContainer.innerHTML = '<span class="status-indicator">CAPTURING 18 PLOT IMAGES AND ANALYZING VECTORS...</span>';

    try {
        const getPlotBase64 = async (elementId) => {
            const el = document.getElementById(elementId);
            if (!el || !el.data) return null;
            const dataUrl = await Plotly.toImage(elementId, { format: 'png', width: 800, height: 250 });
            return dataUrl.split(',')[1];
        };

        const plotIds = [
            'trackMapPlot',
            'frontTempCorneringPlot', 'rearTempCorneringPlot',
            'frontTempStraightPlot', 'rearTempStraightPlot',
            'frontCorneringPlot', 'rearCorneringPlot',
            'frontStraightPlot', 'rearStraightPlot',
            'frontDamperPlot', 'rearDamperPlot',
            'accelFrontDiffPlot', 'accelRearDiffPlot', 'accelCenterDiffPlot',
            'decelFrontDiffPlot', 'decelRearDiffPlot', 'decelCenterDiffPlot',
            'dynoPlot'
        ];

        const images = await Promise.all(plotIds.map(id => getPlotBase64(id)));
        const imageParts = images
            .filter(img => img !== null)
            .map(imgData => ({ inlineData: { mimeType: 'image/png', data: imgData } }));

        const promptText = `
            Analyze the attached Forza Horizon 6 telemetry plots and provide strict relative slider tuning recommendations (e.g., increase, decrease, stiffen, soften). Do not provide specific numerical values.

            OPTIMIZATION TARGETS & DEFINITIONS:
            - Tire Temperature: Target peak operating temperature range is FRONT ${telemetryData.optimal_temp_front.min.toFixed(0)}° to ${telemetryData.optimal_temp_front.max.toFixed(0)}° | REAR ${telemetryData.optimal_temp_rear.min.toFixed(0)}° to ${telemetryData.optimal_temp_rear.max.toFixed(0)}°.
              * FRONT CORNERING: P50=${telemetryData.temp_stats.cornering_front.p50.toFixed(1)}°, MAD=${telemetryData.temp_stats.cornering_front.mad.toFixed(2)}°, In-Zone=${telemetryData.temp_stats.cornering_front.inZonePct}%.
              * REAR CORNERING: P50=${telemetryData.temp_stats.cornering_rear.p50.toFixed(1)}°, MAD=${telemetryData.temp_stats.cornering_rear.mad.toFixed(2)}°, In-Zone=${telemetryData.temp_stats.cornering_rear.inZonePct}%.
              * FRONT STRAIGHT: P50=${telemetryData.temp_stats.straight_front.p50.toFixed(1)}°, MAD=${telemetryData.temp_stats.straight_front.mad.toFixed(2)}°, In-Zone=${telemetryData.temp_stats.straight_front.inZonePct}%.
              * REAR STRAIGHT: P50=${telemetryData.temp_stats.straight_rear.p50.toFixed(1)}°, MAD=${telemetryData.temp_stats.straight_rear.mad.toFixed(2)}°, In-Zone=${telemetryData.temp_stats.straight_rear.inZonePct}%.
              Recommend pressure/spring adjustments if P50 deviates from target or in-zone saturation is low. Compare straight vs cornering to dictate camber adjustments.
            - Normalized Suspension Travel: Target P50 (Median) between 0.45 and 0.50. Target MAD (Median Absolute Deviation) between 0.15 and 0.20. (0 = fully extended, 1 = fully compressed).
              * FRONT CORNERING: P01=${telemetryData.susp_stats.cornering_front.p01.toFixed(3)}, P50=${telemetryData.susp_stats.cornering_front.p50.toFixed(3)}, P99=${telemetryData.susp_stats.cornering_front.p99.toFixed(3)}, MAD=${telemetryData.susp_stats.cornering_front.mad.toFixed(3)}.
              * REAR CORNERING: P01=${telemetryData.susp_stats.cornering_rear.p01.toFixed(3)}, P50=${telemetryData.susp_stats.cornering_rear.p50.toFixed(3)}, P99=${telemetryData.susp_stats.cornering_rear.p99.toFixed(3)}, MAD=${telemetryData.susp_stats.cornering_rear.mad.toFixed(3)}.
              * FRONT STRAIGHT: P01=${telemetryData.susp_stats.straight_front.p01.toFixed(3)}, P50=${telemetryData.susp_stats.straight_front.p50.toFixed(3)}, P99=${telemetryData.susp_stats.straight_front.p99.toFixed(3)}, MAD=${telemetryData.susp_stats.straight_front.mad.toFixed(3)}.
              * REAR STRAIGHT: P01=${telemetryData.susp_stats.straight_rear.p01.toFixed(3)}, P50=${telemetryData.susp_stats.straight_rear.p50.toFixed(3)}, P99=${telemetryData.susp_stats.straight_rear.p99.toFixed(3)}, MAD=${telemetryData.susp_stats.straight_rear.mad.toFixed(3)}.
              Provide spring rate, ride height, bump, and rebound adjustments to achieve these exact statistical bounds.
            - Damper Velocity (30 mm/s Threshold): Target Low-Speed Bump (LSB) 35-40%, Low-Speed Rebound (LSR) 35-40%, High-Speed Bump (HSB) 10-15%, High-Speed Rebound (HSR) 10-15%. NOTE: The game only features single global sliders for "Bump Stiffness" and "Rebound Stiffness". Synthesize the low/high speed metrics to provide single directional adjustments for Bump and Rebound. Do not recommend separate high-speed or low-speed adjustments.
            - Differential Slip Percentiles (Target: P95 between 0.08 and 0.18):
              * ACCELERATION (EXIT): Front P95: ${telemetryData.diff_stats.accel_f.p95.toFixed(3)}, P99: ${telemetryData.diff_stats.accel_f.p99.toFixed(3)} | Rear P95: ${telemetryData.diff_stats.accel_r.p95.toFixed(3)}, P99: ${telemetryData.diff_stats.accel_r.p99.toFixed(3)}
              * DECELERATION (ENTRY): Front P95: ${telemetryData.diff_stats.decel_f.p95.toFixed(3)}, P99: ${telemetryData.diff_stats.decel_f.p99.toFixed(3)} | Rear P95: ${telemetryData.diff_stats.decel_r.p95.toFixed(3)}, P99: ${telemetryData.diff_stats.decel_r.p99.toFixed(3)}
              Recommend decreasing lock (open diff) if P95 < 0.08 (locked/short tail). Recommend increasing lock if P95 > 0.18 or P99 > 0.25 (too open/long tail).
            - Center Differential Directional Mean (Target: balanced power distribution):
              * ACCEL MEAN (μ): ${telemetryData.diff_mean.accel_c.toFixed(3)}
              * DECEL MEAN (μ): ${telemetryData.diff_mean.decel_c.toFixed(3)}
              Recommend decreasing balance (shift torque forward) or increasing center lock if μ < 0 (rear bias / oversteer). Recommend increasing balance (shift torque rearward) if μ > 0 (front bias / understeer).
            - Engine Dyno: Analyze peak HP/TQ curve for gear tuning implications.

            TELEMETRY DATA:
            Drivetrain: ${telemetryData.drivetrain}
            Max HP: ${telemetryData.engine.max_hp.toFixed(1)}
            Max TQ: ${telemetryData.engine.max_tq.toFixed(1)}

            Provide blunt, directive instructions. Use bullet points for structural clarity. Base recommendations strictly on visual and statistical variance in the attached plots against optimization targets.
            DO NOT USE LATEX OR MATH BLOCKS. USE PLAIN TEXT UNICODE EXCLUSIVELY FOR GREEK VARIABLES (μ, σ).
        `;

        const contentsArray = [promptText, ...imageParts];

        const ai = new GoogleGenAI({ apiKey: apiKey });
        const response = await ai.models.generateContent({
            model: 'gemini-flash-latest',
            contents: contentsArray,
            config: {
                temperature: 0.1
            }
        });

        let formattedText = response.text
            .replace(/\$?\\mu\$?/gi, 'μ')
            .replace(/\$?\\sigma\$?/gi, 'σ')
            .replace(/\$/g, '')
            .replace(/^###\s+(.*)$/gm, '<br><strong style="color: #d1d2d5; font-size: 13px;">$1</strong>')
            .replace(/^##\s+(.*)$/gm, '<br><strong style="color: #d1d2d5; font-size: 14px;">$1</strong>')
            .replace(/\*\*(.*?)\*\*/g, '<strong style="color: #d1d2d5;">$1</strong>')
            .replace(/^\s*[\*\-]\s+/gm, '  • ')
            .replace(/^---\s*$/gm, '<hr style="border: none; border-top: 1px solid #23252a; margin: 15px 0;">');

        outContainer.innerHTML = formattedText;
    } catch (err) {
        console.error(err);
        outContainer.innerHTML = `<span style="color: #d95829;">API EXECUTION FAILED: ${err.message}</span>`;
    }
}
