"use client"

import { Nav } from "../components/nav";
import { Footer } from "../components/footer";
import { SpecData } from "../components/specData";
import { SensorData } from "../components/sensorData"
import HyperspectralVideoStream from "../components/hyperspectralVideoStream";
import { Sen } from "next/font/google";

const HYPERSPECTRAL_STREAM = process.env.NEXT_PUBLIC_HYPERSPECTRAL_URL || 'http://localhost:8000/hyperspectral/stream';

export default function Page() {
  return (
  <>
  <Nav />
  <hr/>
  <div className = "header">
    <h1 className="homepage-header"> La Princesa Rover </h1>
      <h2 className="homepage-subheader"> Science Payload </h2>
</div>

<h3 className="video-title">Hyperspectral Camera</h3>
<div className = "stream-align">
  <div className="hyperspec">
    <HyperspectralVideoStream 
      streamId="Hyperspectral Camera" 
      streamUrl={HYPERSPECTRAL_STREAM}
      width={1024}
      height={768}
    />
  </div>

  <div className="dataHolder">
    <SensorData />
    <SpecData />
  </div>
  </div>
  
  
    <Footer />
  </>
)
}