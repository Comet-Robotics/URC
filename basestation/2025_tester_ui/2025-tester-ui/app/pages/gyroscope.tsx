"use client"

import { Nav } from "../components/nav";
import { Footer } from "../components/footer";
import GyroscopeVisualizer from "../components/gyroscopeVisualizer";

export default function Page() {
  return (
    <>
<Nav />

      <hr/>

      <div className = "header">
        <h1 className = "homepage-header"> La Princesa Rover </h1>
          <h2 className = "homepage-subheader"> Solis Rover Project</h2>
      </div>

      <h3 className="video-title">Gyroscope Visualization</h3>

      <div className = "stream-align">
        <div className="hyperspec">
          <GyroscopeVisualizer />
        </div>
      </div>

      <Footer />
    </>
)
}