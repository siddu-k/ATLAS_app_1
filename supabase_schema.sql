-- =====================================================
-- ATLAS ROVER - SUPABASE DATABASE SCHEMA
-- =====================================================

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- =====================================================
-- TABLE: rover_paths
-- Stores saved navigation paths with waypoints
-- =====================================================
CREATE TABLE IF NOT EXISTS rover_paths (
    id UUID DEFAULT uuid_generate_v4() PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    description TEXT,
    waypoints JSONB NOT NULL,  -- Array of {lat, lng, order}
    total_distance_km DECIMAL(10, 4),
    estimated_time_min INTEGER,
    speed DECIMAL(4, 2) DEFAULT 0.5,  -- m/s
    arrival_radius DECIMAL(4, 2) DEFAULT 2.0,  -- meters
    is_loop BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    created_by VARCHAR(255) DEFAULT 'ATLAS_OPERATOR',
    is_active BOOLEAN DEFAULT TRUE
);

-- Index for faster queries
CREATE INDEX idx_rover_paths_name ON rover_paths(name);
CREATE INDEX idx_rover_paths_created_at ON rover_paths(created_at DESC);

-- =====================================================
-- TABLE: rover_location
-- Stores real-time and historical rover locations
-- =====================================================
CREATE TABLE IF NOT EXISTS rover_location (
    id UUID DEFAULT uuid_generate_v4() PRIMARY KEY,
    latitude DECIMAL(10, 7) NOT NULL,
    longitude DECIMAL(10, 7) NOT NULL,
    altitude DECIMAL(8, 2),
    heading DECIMAL(5, 2),  -- 0-360 degrees
    speed DECIMAL(6, 2),    -- m/s
    accuracy DECIMAL(6, 2), -- GPS accuracy in meters
    battery_percent INTEGER,
    satellites INTEGER,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    is_latest BOOLEAN DEFAULT FALSE,
    session_id VARCHAR(100)
);

-- Index for location queries
CREATE INDEX idx_rover_location_timestamp ON rover_location(timestamp DESC);
CREATE INDEX idx_rover_location_latest ON rover_location(is_latest) WHERE is_latest = TRUE;

-- =====================================================
-- TABLE: rover_telemetry
-- Stores sensor data history (optional, for analytics)
-- =====================================================
CREATE TABLE IF NOT EXISTS rover_telemetry (
    id UUID DEFAULT uuid_generate_v4() PRIMARY KEY,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- IMU Data
    acc_x DECIMAL(8, 4),
    acc_y DECIMAL(8, 4),
    acc_z DECIMAL(8, 4),
    gyro_x DECIMAL(8, 4),
    gyro_y DECIMAL(8, 4),
    gyro_z DECIMAL(8, 4),
    
    -- GPS
    latitude DECIMAL(10, 7),
    longitude DECIMAL(10, 7),
    heading DECIMAL(5, 2),
    
    -- System
    battery_percent INTEGER,
    wifi_rssi INTEGER,
    
    -- Ultrasonic distances (cm)
    dist_front DECIMAL(6, 2),
    dist_front_left DECIMAL(6, 2),
    dist_front_right DECIMAL(6, 2),
    dist_back_left DECIMAL(6, 2),
    dist_back_right DECIMAL(6, 2)
);

-- Partition by time for better performance (optional)
CREATE INDEX idx_rover_telemetry_timestamp ON rover_telemetry(timestamp DESC);

-- =====================================================
-- TABLE: face_detections
-- Logs face detection events
-- =====================================================
CREATE TABLE IF NOT EXISTS face_detections (
    id UUID DEFAULT uuid_generate_v4() PRIMARY KEY,
    target_name VARCHAR(255) NOT NULL,
    confidence DECIMAL(5, 2),
    latitude DECIMAL(10, 7),
    longitude DECIMAL(10, 7),
    detected_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    image_url TEXT,
    acknowledged BOOLEAN DEFAULT FALSE
);

CREATE INDEX idx_face_detections_time ON face_detections(detected_at DESC);
CREATE INDEX idx_face_detections_name ON face_detections(target_name);

-- =====================================================
-- TABLE: mission_logs
-- Stores mission activity logs
-- =====================================================
CREATE TABLE IF NOT EXISTS mission_logs (
    id UUID DEFAULT uuid_generate_v4() PRIMARY KEY,
    event_type VARCHAR(50) NOT NULL,  -- 'PATH_START', 'PATH_COMPLETE', 'OBSTACLE', 'TARGET_DETECTED', etc.
    message TEXT,
    data JSONB,
    latitude DECIMAL(10, 7),
    longitude DECIMAL(10, 7),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_mission_logs_type ON mission_logs(event_type);
CREATE INDEX idx_mission_logs_time ON mission_logs(created_at DESC);

-- =====================================================
-- FUNCTIONS
-- =====================================================

-- Function to update rover location (sets is_latest flag)
CREATE OR REPLACE FUNCTION update_rover_location(
    p_latitude DECIMAL,
    p_longitude DECIMAL,
    p_heading DECIMAL DEFAULT NULL,
    p_speed DECIMAL DEFAULT NULL,
    p_battery INTEGER DEFAULT NULL,
    p_satellites INTEGER DEFAULT NULL,
    p_session_id VARCHAR DEFAULT NULL
)
RETURNS UUID AS $$
DECLARE
    new_id UUID;
BEGIN
    -- Clear previous latest flag
    UPDATE rover_location SET is_latest = FALSE WHERE is_latest = TRUE;
    
    -- Insert new location
    INSERT INTO rover_location (
        latitude, longitude, heading, speed, 
        battery_percent, satellites, is_latest, session_id
    ) VALUES (
        p_latitude, p_longitude, p_heading, p_speed,
        p_battery, p_satellites, TRUE, p_session_id
    ) RETURNING id INTO new_id;
    
    RETURN new_id;
END;
$$ LANGUAGE plpgsql;

-- Function to get latest rover location
CREATE OR REPLACE FUNCTION get_latest_rover_location()
RETURNS TABLE (
    latitude DECIMAL,
    longitude DECIMAL,
    heading DECIMAL,
    speed DECIMAL,
    battery_percent INTEGER,
    satellites INTEGER,
    last_updated TIMESTAMP WITH TIME ZONE
) AS $$
BEGIN
    RETURN QUERY
    SELECT 
        rl.latitude, rl.longitude, rl.heading, rl.speed,
        rl.battery_percent, rl.satellites, rl.timestamp
    FROM rover_location rl
    WHERE rl.is_latest = TRUE
    LIMIT 1;
END;
$$ LANGUAGE plpgsql;

-- =====================================================
-- ROW LEVEL SECURITY (RLS) - Optional
-- =====================================================
-- Enable RLS on tables
ALTER TABLE rover_paths ENABLE ROW LEVEL SECURITY;
ALTER TABLE rover_location ENABLE ROW LEVEL SECURITY;
ALTER TABLE rover_telemetry ENABLE ROW LEVEL SECURITY;

-- Policy: Allow all operations for authenticated users (adjust as needed)
CREATE POLICY "Allow all for authenticated" ON rover_paths
    FOR ALL USING (true);

CREATE POLICY "Allow all for authenticated" ON rover_location
    FOR ALL USING (true);

CREATE POLICY "Allow all for authenticated" ON rover_telemetry
    FOR ALL USING (true);

-- =====================================================
-- TABLE: settings
-- Single-row table for system configuration
-- =====================================================
CREATE TABLE IF NOT EXISTS settings (
    id INTEGER PRIMARY KEY CHECK (id = 1), -- Single row enforcement
    rover_ip VARCHAR(50),
    mobile_ip VARCHAR(50),
    video_rotation INTEGER DEFAULT 0,
    gun_speed INTEGER DEFAULT 255,
    heading_offset INTEGER DEFAULT 0,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Insert default row if not exists
INSERT INTO settings (id, rover_ip, mobile_ip, video_rotation, gun_speed, heading_offset)
VALUES (1, '192.168.1.100', '10.49.229.251', 0, 255, 0)
ON CONFLICT (id) DO NOTHING;

-- Enable RLS
ALTER TABLE settings ENABLE ROW LEVEL SECURITY;

-- Policy (adjust as needed for authenticated/anon access)
CREATE POLICY "Allow all public access to settings" ON settings
    FOR ALL USING (true);

-- =====================================================
-- CLEANUP OLD DATA (Optional scheduled function)
-- =====================================================
-- Function to clean old telemetry data (keep last 7 days)
CREATE OR REPLACE FUNCTION cleanup_old_telemetry()
RETURNS void AS $$
BEGIN
    DELETE FROM rover_telemetry 
    WHERE timestamp < NOW() - INTERVAL '7 days';
    
    DELETE FROM rover_location 
    WHERE is_latest = FALSE 
    AND timestamp < NOW() - INTERVAL '30 days';
END;
$$ LANGUAGE plpgsql;

-- =====================================================
-- SAMPLE DATA (for testing)
-- =====================================================
-- Insert a sample path
INSERT INTO rover_paths (name, description, waypoints, total_distance_km, estimated_time_min, speed, is_loop)
VALUES (
    'Test Mission Alpha',
    'Sample patrol route around the perimeter',
    '[
        {"lat": 12.9716, "lng": 77.5946, "order": 1},
        {"lat": 12.9720, "lng": 77.5950, "order": 2},
        {"lat": 12.9725, "lng": 77.5945, "order": 3},
        {"lat": 12.9718, "lng": 77.5940, "order": 4}
    ]'::jsonb,
    0.85,
    17,
    0.5,
    TRUE
);

-- =====================================================
-- VIEWS (Optional - for dashboard queries)
-- =====================================================
CREATE OR REPLACE VIEW v_latest_rover_status AS
SELECT 
    rl.latitude,
    rl.longitude,
    rl.heading,
    rl.speed,
    rl.battery_percent,
    rl.satellites,
    rl.timestamp as last_update,
    EXTRACT(EPOCH FROM (NOW() - rl.timestamp)) as seconds_since_update
FROM rover_location rl
WHERE rl.is_latest = TRUE;

CREATE OR REPLACE VIEW v_recent_detections AS
SELECT 
    fd.target_name,
    fd.confidence,
    fd.latitude,
    fd.longitude,
    fd.detected_at,
    fd.acknowledged
FROM face_detections fd
WHERE fd.detected_at > NOW() - INTERVAL '24 hours'
ORDER BY fd.detected_at DESC;
