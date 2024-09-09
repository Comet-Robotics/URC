import { useEffect, useRef } from 'react';
import Hls from 'hls.js';
//WEB RTC WOULD BE BETTER
const HLSVideo = ({ videoSrc }:{videoSrc: string}) => {
    const videoRef = useRef<HTMLVideoElement>(null);

    useEffect(() => {
        if (Hls.isSupported() && videoRef.current) {
            const hls = new Hls({
                liveSyncDuration: 1,  // Target 1 second of buffer behind live edge
                liveMaxLatencyDuration: 2,  // Max allowed latency behind the live edge
                maxLiveSyncPlaybackRate: 1.5,  // Speed up playback to catch up if falling behind
                lowLatencyMode: true,  // Enable low-latency HLS (if supported by the server)
            });
            hls.loadSource(videoSrc);
            hls.attachMedia(videoRef.current);
            hls.on(Hls.Events.MANIFEST_PARSED, () => {

                videoRef?.current?.play();
            });

            // Optional: Ensure we are as close to the live edge as possible
            hls.on(Hls.Events.FRAG_LOADED, () => {
                if (videoRef.current && !videoRef.current.seeking) {
                    const latency = hls.latency;
                    const drift = hls.drift;
                    console.log(`Current latency: ${latency}, drift: ${drift}`);

                    // Check how far behind we are from the live edge and seek if necessary
                    const liveEdge = hls.liveSyncPosition;
                    const currentPosition = videoRef.current.currentTime;
                    if (liveEdge && liveEdge - currentPosition > 1.0) {
                        videoRef.current.currentTime = liveEdge;
                    }
                }
            });
        } else if (videoRef.current && videoRef.current.canPlayType('application/vnd.apple.mpegurl')) {
            videoRef.current.src = videoSrc;
            videoRef.current.addEventListener('canplay', () => {
                videoRef?.current?.play();
            });
        }
    }, [videoSrc,videoRef]);

    return (
        <video ref={videoRef} controls width="640" height="360" autoPlay />
    );
};

export default HLSVideo;
