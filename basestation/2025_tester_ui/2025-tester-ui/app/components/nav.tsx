// The Navbar Component

import { NavLink } from "react-router-dom";

// The function that renders the Navbar

export function Nav() {
  return (
    <nav>

        {/* Navigation links */}

        <NavLink to="/" className={"navItems"} >Home</NavLink>
        <NavLink to="/science-payload" className={"navItems"}>Science Payload</NavLink>
        <NavLink to="/gyroscope" className={"navItems"}>Gyroscope</NavLink>
    </nav>
  );
}   