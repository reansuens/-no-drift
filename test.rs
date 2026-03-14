    loop {
        info!("CALIBRATION: STARTING 5s UNIFIED TELEMETRY ARRAY");

        // Initialize Encoder Accumulators
        let mut edges_la: u32 = 0;
        let mut edges_lb: u32 = 0;
        let mut edges_ra: u32 = 0;
        let mut edges_rb: u32 = 0;

        let mut prev_la = encoders.left_a.is_high();
        let mut prev_lb = encoders.left_b.is_high();
        let mut prev_ra = encoders.right_a.is_high();
        let mut prev_rb = encoders.right_b.is_high();

        // Initiate Forward Motion Vector
        drive.execute(VehicleMotion::Forward, 100, 50);

        // UNIFIED SAMPLING CORE: 500 iterations * 10ms = 5000ms
        for i in 0..500 {
            let (ax, ay, az, gz) = mpu.read_corrected(&bias);

            // High-frequency telemetry logging (Every 50ms)
            if i % 5 == 0 {
                info!("[T+{}ms] AX: {} | AY: {} | GZ: {}", i * 10, ax, ay, gz);
            }

            // Encoder Edge Detection Logic
            let now_la = encoders.left_a.is_high();
            let now_lb = encoders.left_b.is_high();
            let now_ra = encoders.right_a.is_high();
            let now_rb = encoders.right_b.is_high();

            if now_la != prev_la {
                edges_la += 1;
                prev_la = now_la;
            }
            if now_lb != prev_lb {
                edges_lb += 1;
                prev_lb = now_lb;
            }
            if now_ra != prev_ra {
                edges_ra += 1;
                prev_ra = now_ra;
            }
            if now_rb != prev_rb {
                edges_rb += 1;
                prev_rb = now_rb;
            }

            delay.delay_millis(10);
        }

        // Terminate Motion Vector
        drive.execute(VehicleMotion::Stop, 0, 0);

        // FINAL DATA PAYLOAD DELIVERY
        info!("--- CALIBRATION COMPLETE ---");
        info!("R_A:     {}", edges_ra);
        info!("R_B:     {}", edges_rb);
        info!("R_Total: {}", edges_ra + edges_rb);
        info!("L_A:     {}", edges_la);
        info!("L_B:     {}", edges_lb);
        info!("L_Total: {}", edges_la + edges_lb);
        info!("SYSTEM STATUS: IDLE");

        // Terminal Idle Mode
        loop {
            delay.delay_millis(1000);
        }
    }
