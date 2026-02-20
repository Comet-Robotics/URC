"use client"

import React, { useEffect, useRef, useState } from 'react';

interface IMUData {
  accel: { x: number; y: number; z: number };
  gyro: { x: number; y: number; z: number };
  orientation: { pitch: number; roll: number; yaw: number };
  timestamp: number;
}

const GyroscopeVisualizer: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [connected, setConnected] = useState(false);
  const [wsError, setWsError] = useState<string | null>(null);
  const [imuData, setImuData] = useState<IMUData>({
    accel: { x: 0, y: 0, z: 0 },
    gyro: { x: 0, y: 0, z: 0 },
    orientation: { pitch: 0, roll: 0, yaw: 0 },
    timestamp: 0
  });
  const wsRef = useRef<WebSocket | null>(null);
  const frameCountRef = useRef(0);

  // Draw 3D cube with rotation based on orientation
  const drawCube = (
    ctx: CanvasRenderingContext2D,
    pitch: number,
    roll: number,
    yaw: number
  ) => {
    const width = canvasRef.current?.width || 600;
    const height = canvasRef.current?.height || 600;

    // Clear canvas
    ctx.fillStyle = '#2d2d2d';
    ctx.fillRect(0, 0, width, height);

    // Set up 3D context
    ctx.save();
    ctx.translate(width / 2, height / 2);

    // Convert degrees to radians
    const pitchRad = (pitch * Math.PI) / 180;
    const rollRad = (roll * Math.PI) / 180;
    const yawRad = (yaw * Math.PI) / 180;

    // Draw cube with faces
    const scale = 100;
    const vertices = [
      [-1, -1, -1],
      [1, -1, -1],
      [1, 1, -1],
      [-1, 1, -1],
      [-1, -1, 1],
      [1, -1, 1],
      [1, 1, 1],
      [-1, 1, 1]
    ];

    // Apply rotations
    const rotatedVertices = vertices.map(([x, y, z]) => {
      // Roll rotation
      const cosRoll = Math.cos(rollRad);
      const sinRoll = Math.sin(rollRad);
      const y1 = y * cosRoll - z * sinRoll;
      const z1 = y * sinRoll + z * cosRoll;

      // Pitch rotation
      const cosPitch = Math.cos(pitchRad);
      const sinPitch = Math.sin(pitchRad);
      const z2 = z1 * cosPitch - x * sinPitch;
      const x2 = z1 * sinPitch + x * cosPitch;

      // Yaw rotation
      const cosYaw = Math.cos(yawRad);
      const sinYaw = Math.sin(yawRad);
      const x3 = x2 * cosYaw - y1 * sinYaw;
      const y3 = x2 * sinYaw + y1 * cosYaw;

      return [x3 * scale, y3 * scale, z2 * scale];
    });

    // Project 3D points to 2D
    const projectedVertices = rotatedVertices.map(([x, y, z]) => {
      const distance = 5;
      const scale2d = distance / (distance + z / 100);
      return [x * scale2d, y * scale2d, z];
    });

    // Define faces with colors
    const faces = [
      { vertices: [0, 1, 2, 3], color: '#ff4444', label: 'Front' },
      { vertices: [4, 5, 6, 7], color: '#4444ff', label: 'Back' },
      { vertices: [0, 1, 5, 4], color: '#44ff44', label: 'Bottom' },
      { vertices: [2, 3, 7, 6], color: '#ffff44', label: 'Top' },
      { vertices: [0, 3, 7, 4], color: '#ff44ff', label: 'Left' },
      { vertices: [1, 2, 6, 5], color: '#44ffff', label: 'Right' }
    ];

    // Sort faces by average Z for depth sorting
    const facesWithZ = faces.map(face => ({
      ...face,
      avgZ: face.vertices.reduce((sum, idx) => sum + projectedVertices[idx][2], 0) / face.vertices.length
    }));

    facesWithZ.sort((a, b) => a.avgZ - b.avgZ);

    // Draw faces
    facesWithZ.forEach(face => {
      ctx.fillStyle = face.color;
      ctx.globalAlpha = 0.7;
      ctx.beginPath();
      const [x0, y0] = projectedVertices[face.vertices[0]];
      ctx.moveTo(x0, y0);
      for (let i = 1; i < face.vertices.length; i++) {
        const [x, y] = projectedVertices[face.vertices[i]];
        ctx.lineTo(x, y);
      }
      ctx.closePath();
      ctx.fill();
      ctx.globalAlpha = 1;
    });

    // Draw edges
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;

    const edges = [
      [0, 1],
      [1, 2],
      [2, 3],
      [3, 0],
      [4, 5],
      [5, 6],
      [6, 7],
      [7, 4],
      [0, 4],
      [1, 5],
      [2, 6],
      [3, 7]
    ];

    edges.forEach(([start, end]) => {
      const [x1, y1] = projectedVertices[start];
      const [x2, y2] = projectedVertices[end];
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    });

    // Draw axes
    ctx.font = 'bold 16px Arial';
    ctx.lineWidth = 3;

    // X axis (red)
    ctx.strokeStyle = '#ff0000';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(120, 0);
    ctx.stroke();
    ctx.fillStyle = '#ff0000';
    ctx.fillText('X', 130, 5);

    // Y axis (green)
    ctx.strokeStyle = '#00ff00';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, 120);
    ctx.stroke();
    ctx.fillStyle = '#00ff00';
    ctx.fillText('Y', 5, 135);

    // Z axis (blue)
    ctx.strokeStyle = '#0000ff';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(85, -85);
    ctx.stroke();
    ctx.fillStyle = '#0000ff';
    ctx.fillText('Z', 95, -95);

    ctx.restore();
  };

  // Animation loop for cube
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const animate = () => {
      drawCube(ctx, imuData.orientation.pitch, imuData.orientation.roll, imuData.orientation.yaw);
      frameCountRef.current = requestAnimationFrame(animate);
    };

    animate();

    return () => cancelAnimationFrame(frameCountRef.current);
  }, [imuData]);

  // WebSocket connection
  useEffect(() => {
    const connectWebSocket = () => {
      try {
        const foxgloveUrl = process.env.NEXT_PUBLIC_FOXGLOVE_WS_URL || 'ws://192.168.1.100:8765';
        const ws = new WebSocket(foxgloveUrl);

        ws.onopen = () => {
          console.log('WebSocket connected to Foxglove server');
          setConnected(true);
          setWsError(null);
        };

        ws.onerror = (error) => {
          console.error('WebSocket error:', error);
          const foxgloveUrl = process.env.NEXT_PUBLIC_FOXGLOVE_WS_URL || 'ws://192.168.1.100:8765';
          setWsError(`Failed to connect to Foxglove server at ${foxgloveUrl}`);
          setConnected(false);
        };

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);

            // Check if it contains IMU data
            if (data.accel || data.gyro || data.orientation) {
              const newData = {
                accel: data.accel || imuData.accel,
                gyro: data.gyro || imuData.gyro,
                orientation: data.orientation || imuData.orientation,
                timestamp: Date.now()
              };

              setImuData(newData);
            }
          } catch (error) {
            console.error('Error parsing WebSocket message:', error);
          }
        };

        ws.onclose = () => {
          console.log('WebSocket disconnected');
          setConnected(false);
          // Try to reconnect after 3 seconds
          setTimeout(connectWebSocket, 3000);
        };

        wsRef.current = ws;
      } catch (error) {
        console.error('WebSocket connection error:', error);
        setWsError('Failed to establish WebSocket connection');
      }
    };

    connectWebSocket();

    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

  return (
    <div className="gyro-container">
      <style>{`
        .gyro-container {
          width: 100%;
          height: 100%;
          display: flex;
          flex-direction: column;
          background: linear-gradient(135deg, #2d2d2d 0%, #1a1a1a 100%);
          border-radius: 8px;
          overflow: hidden;
          font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
          color: #e0e0e0;
        }

        .gyro-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 12px 16px;
          background-color: rgba(0, 0, 0, 0.3);
          border-bottom: 1px solid #404040;
          flex-shrink: 0;
        }

        .status-badge {
          display: flex;
          align-items: center;
          gap: 8px;
          font-size: 13px;
          font-weight: 600;
        }

        .status-dot {
          width: 10px;
          height: 10px;
          border-radius: 50%;
          background-color: ${connected ? '#4ade80' : '#ef4444'};
          box-shadow: 0 0 8px ${connected ? '#4ade80' : '#ef4444'};
          animation: pulse 2s infinite;
        }

        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.6; }
        }

        .gyro-content {
          display: flex;
          flex: 1;
          overflow: hidden;
          gap: 12px;
          padding: 12px;
        }

        .cube-section {
          flex: 1;
          display: flex;
          flex-direction: column;
          background-color: rgba(45, 45, 45, 0.6);
          border: 1px solid #404040;
          border-radius: 6px;
          overflow: hidden;
        }

        .cube-canvas {
          flex: 1;
          width: 100%;
          height: 100%;
          display: block;
          background: linear-gradient(to bottom, #2d2d2d, #1a1a1a);
        }

        .sidebar {
          display: flex;
          flex-direction: column;
          gap: 10px;
          width: 220px;
          overflow-y: auto;
        }

        .data-card {
          background-color: rgba(45, 45, 45, 0.6);
          border: 1px solid #404040;
          border-radius: 6px;
          padding: 10px 12px;
          flex-shrink: 0;
        }

        .data-card-title {
          font-size: 11px;
          font-weight: 700;
          color: #64b5f6;
          text-transform: uppercase;
          letter-spacing: 0.5px;
          margin-bottom: 8px;
          border-bottom: 1px solid #404040;
          padding-bottom: 6px;
        }

        .data-values {
          display: grid;
          grid-template-columns: 1fr 1fr;
          gap: 6px;
        }

        .value-item {
          background-color: rgba(0, 0, 0, 0.3);
          border-left: 2px solid #64b5f6;
          padding: 6px 8px;
          border-radius: 3px;
        }

        .value-label {
          font-size: 10px;
          color: #90caf9;
          text-transform: uppercase;
          font-weight: 600;
          margin-bottom: 2px;
        }

        .value-number {
          font-size: 14px;
          font-weight: bold;
          color: #64b5f6;
          font-family: 'Courier New', monospace;
        }

        .value-unit {
          font-size: 9px;
          color: #666;
          margin-left: 2px;
        }

        .orientation-card {
          display: flex;
          gap: 8px;
          padding: 10px 12px;
          background-color: rgba(45, 45, 45, 0.6);
          border: 1px solid #404040;
          border-radius: 6px;
        }

        .angle-item {
          flex: 1;
          text-align: center;
          background-color: rgba(0, 0, 0, 0.3);
          border-radius: 4px;
          padding: 8px 6px;
        }

        .angle-label {
          font-size: 10px;
          color: #90caf9;
          font-weight: 600;
          text-transform: uppercase;
          margin-bottom: 4px;
        }

        .angle-value {
          font-size: 16px;
          font-weight: bold;
          font-family: 'Courier New', monospace;
        }

        .angle-value.pitch { color: #ff6b6b; }
        .angle-value.roll { color: #51cf66; }
        .angle-value.yaw { color: #4dabf7; }

        .error-message {
          color: #ff6b6b;
          font-size: 11px;
          padding: 8px 12px;
          background-color: rgba(255, 107, 107, 0.1);
          border-left: 2px solid #ff6b6b;
          border-radius: 3px;
          margin-top: 8px;
        }

        @media (max-width: 1200px) {
          .gyro-content {
            flex-direction: column;
          }

          .sidebar {
            width: 100%;
            flex-direction: row;
            flex-wrap: wrap;
            max-height: 120px;
          }

          .data-card {
            flex: 1;
            min-width: 150px;
          }
        }

        .sidebar::-webkit-scrollbar {
          width: 6px;
        }

        .sidebar::-webkit-scrollbar-track {
          background: rgba(0, 0, 0, 0.2);
          border-radius: 3px;
        }

        .sidebar::-webkit-scrollbar-thumb {
          background: #404040;
          border-radius: 3px;
        }

        .sidebar::-webkit-scrollbar-thumb:hover {
          background: #505050;
        }
      `}</style>

      <div className="gyro-header">
        <div className="status-badge">
          <div className="status-dot"></div>
          <span>{connected ? 'Connected' : 'Connecting...'}</span>
        </div>
        {imuData.timestamp && (
          <span style={{ fontSize: '12px', color: '#999' }}>
            {new Date(imuData.timestamp).toLocaleTimeString()}
          </span>
        )}
      </div>

      {wsError && (
        <div style={{ padding: '0 12px', paddingTop: '12px' }}>
          <div className="error-message">⚠️ {wsError}</div>
        </div>
      )}

      <div className="gyro-content">
        <div className="cube-section">
          <canvas
            ref={canvasRef}
            width={600}
            height={600}
            className="cube-canvas"
          />
        </div>

        <div className="sidebar">
          {/* Orientation Angles */}
          <div className="orientation-card">
            <div className="angle-item">
              <div className="angle-label">Pitch</div>
              <div className="angle-value pitch">
                {imuData.orientation.pitch.toFixed(1)}°
              </div>
            </div>
            <div className="angle-item">
              <div className="angle-label">Roll</div>
              <div className="angle-value roll">
                {imuData.orientation.roll.toFixed(1)}°
              </div>
            </div>
            <div className="angle-item">
              <div className="angle-label">Yaw</div>
              <div className="angle-value yaw">
                {imuData.orientation.yaw.toFixed(1)}°
              </div>
            </div>
          </div>

          {/* Accelerometer */}
          <div className="data-card">
            <div className="data-card-title">Accelerometer</div>
            <div className="data-values">
              <div className="value-item">
                <div className="value-label">X</div>
                <div className="value-number">
                  {imuData.accel.x.toFixed(1)}
                  <span className="value-unit">g</span>
                </div>
              </div>
              <div className="value-item">
                <div className="value-label">Y</div>
                <div className="value-number">
                  {imuData.accel.y.toFixed(1)}
                  <span className="value-unit">g</span>
                </div>
              </div>
              <div className="value-item">
                <div className="value-label">Z</div>
                <div className="value-number">
                  {imuData.accel.z.toFixed(1)}
                  <span className="value-unit">g</span>
                </div>
              </div>
            </div>
          </div>

          {/* Gyroscope */}
          <div className="data-card">
            <div className="data-card-title">Gyroscope</div>
            <div className="data-values">
              <div className="value-item">
                <div className="value-label">X</div>
                <div className="value-number">
                  {imuData.gyro.x.toFixed(0)}
                  <span className="value-unit">°/s</span>
                </div>
              </div>
              <div className="value-item">
                <div className="value-label">Y</div>
                <div className="value-number">
                  {imuData.gyro.y.toFixed(0)}
                  <span className="value-unit">°/s</span>
                </div>
              </div>
              <div className="value-item">
                <div className="value-label">Z</div>
                <div className="value-number">
                  {imuData.gyro.z.toFixed(0)}
                  <span className="value-unit">°/s</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default GyroscopeVisualizer;
