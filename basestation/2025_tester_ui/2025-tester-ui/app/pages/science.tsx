"use client"

import { Nav } from "../components/nav";
import { Footer } from "../components/footer"
import HyperspectralVideoStream from "../components/hyperspectralVideoStream";

const HYPERSPECTRAL_STREAM = process.env.NEXT_PUBLIC_HYPERSPECTRAL_URL || 'http://localhost:8000/hyperspectral/stream';

export default function Page() {
  return (
  <>
  <Nav />
  <hr/>
  <div className = "header">
    <h1> La Princesa Rover </h1>
      <h2> Science Payload </h2>
  <p>science.tsx</p>

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
</div>
  </div>
  
    <Footer />
  </>
)
}