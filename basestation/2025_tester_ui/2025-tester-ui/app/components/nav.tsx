import { NavLink } from "react-router-dom";

export function Nav() {
  return (
    <nav>
        <NavLink to="/homepage">Home</NavLink>
        <NavLink to="/science-payload">Science Payload</NavLink>
        <NavLink to="/gyroscope">Gyroscope</NavLink>
    </nav>
  );
}   