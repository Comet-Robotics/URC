"use client"

import { Nav } from "../components/nav";

export default function Page() {
  return (
  <>
  <Nav />
  <hr/>
  <div className = "header">
    <h1> La Princesa Rover </h1>
      <h2> Gyroscope </h2>
      <p>gyroscope.tsx</p>
  </div>

  <div className="centering-block">
      <div className = "block">
        <p> La Princesa Rover </p>
      </div>

      <div className = "block">
        <p> La Princesa Rover 2</p>
      </div>
  </div>
  

  </>
)
}