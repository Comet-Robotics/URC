// Header component

import { Nav } from "./nav";

// The function that renders the Header

export function Header() {
  return (
    <div className="header">
      <h1>Solis Rover Project</h1>
      <Nav />
      {/* Other stuff for the header here */}
    </div>
  );
}