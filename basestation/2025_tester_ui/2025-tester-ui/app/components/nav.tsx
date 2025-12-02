// The Navbar Component

import { NavLink } from "react-router-dom";

// The function that renders the Navbar

export function Nav() {
  return (
    <nav>

        {/* Navigation links */}

        <NavLink to="/">Home</NavLink>
        <NavLink to="/science-payload">Science Payload</NavLink>
        <NavLink to="/gyroscope">Gyroscope</NavLink>
    </nav>
  );
}   