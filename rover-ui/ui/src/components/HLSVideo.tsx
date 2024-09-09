import { useEffect, useRef } from 'react';
import Hls from 'hls.js';

const HLSVideo = ({ videoSrc }) => {
    const videoRef = useRef(null);

    useEffect(() => {
        if (Hls.isSupported()) {
            const hls = new Hls();
            hls.loadSource(videoSrc);
            hls.attachMedia(videoRef.current);
            hls.on(Hls.Events.MANIFEST_PARSED, () => {
                videoRef.current.play();
                });
            } else if (videoRef.current.canPlayType('application/vnd.apple.mpegurl')) {
            videoRef.current.src = videoSrc;
            videoRef.current.addEventListener('canplay', () => {
                videoRef.current.play();
                });
            }
        }, [videoSrc]);

    return (
        <video ref={videoRef} controls width="640" height="360" autoPlay />
    );
};

export default HLSVideo;
