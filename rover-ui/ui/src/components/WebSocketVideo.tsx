import { useEffect, useState, useRef } from "react";

const WebSocketVideo = ({ wsUrl = "/ws", width = 640, height = 480 }) => {
    const [isConnected, setIsConnected] = useState(false);
    const [frameSrc, setFrameSrc] = useState<string |null>(null); // Holds the image URL for the current frame
    const wsRef = useRef<WebSocket|null>(null); // Reference to the WebSocket instance

    useEffect(() => {
        // Create WebSocket connection.
        const ws = new WebSocket(wsUrl);
        wsRef.current = ws;

        ws.onopen = () => {
            console.log("WebSocket connection established");
            setIsConnected(true);
        };

        ws.onclose = () => {
            console.log("WebSocket connection closed");
            setIsConnected(false);
        };

        ws.onerror = (error) => {
            console.error("WebSocket error", error);
            setIsConnected(false);
        };

        ws.onmessage = (event) => {
            // Create a Blob URL from the received binary data (assuming JPEG frames)
            const blob = new Blob([event.data], { type: "image/jpeg" });
            const imageUrl = URL.createObjectURL(blob);
            setFrameSrc(imageUrl);

            // Free up the memory for the previous frame
            if (frameSrc) {
                URL.revokeObjectURL(frameSrc);
            }
        };

        // Cleanup WebSocket when the component unmounts
        return () => {
            ws.close();
        };
    }, [wsUrl]);

    return (
        <div>
            <h2>WebSocket Video Stream</h2>
            {isConnected ? (
                <img src={frameSrc!} width={width} height={height} alt="Video Stream" />
            ) : (
                <p>Connecting to video stream...</p>
            )}
        </div>
    );
};

export default WebSocketVideo;
