function Sw({ children }: { children: React.ReactNode }) {
  return (
    <svg width="20" height="14" viewBox="0 0 22 16" aria-hidden="true" className="shrink-0">
      {children}
    </svg>
  );
}

function Item({ sw, children }: { sw: React.ReactNode; children: React.ReactNode }) {
  return (
    <li className="flex min-w-0 items-center gap-1.5">
      {sw}
      <span>{children}</span>
    </li>
  );
}

/** Map legend as an evenly spaced two-column grid. Every colour has a text/icon backup. */
export function LegendGrid() {
  return (
    <ul className="grid grid-cols-1 gap-x-4 gap-y-1 text-2xs text-muted sm:grid-cols-2" aria-label="Map legend">
      <Item sw={<Sw><rect x="1" y="2" width="20" height="12" fill="var(--m-free)" stroke="var(--m-free-edge)" strokeWidth="1.5" /></Sw>}>Aisle free</Item>
      <Item sw={<Sw><rect x="1" y="2" width="20" height="12" fill="#4aa3ff" fillOpacity=".3" stroke="#4aa3ff" strokeWidth="1.5" /></Sw>}>Held (🔒R#)</Item>
      <Item sw={<Sw><rect x="1" y="2" width="20" height="12" fill="none" stroke="var(--c-warn)" strokeWidth="2" strokeDasharray="4 2" /></Sw>}>⏳ Queued</Item>
      <Item sw={<Sw><defs><pattern id="lg-hatch" width="4" height="4" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><rect width="4" height="4" fill="var(--m-unknown)" /><line x1="0" y1="0" x2="0" y2="4" stroke="var(--c-unknown)" strokeWidth="1.2" /></pattern></defs><rect x="1" y="2" width="20" height="12" fill="url(#lg-hatch)" stroke="var(--c-unknown)" strokeWidth="1.5" /></Sw>}>? Unknown</Item>
      <Item sw={<Sw><rect x="1" y="2" width="20" height="12" fill="none" stroke="var(--c-crit)" strokeWidth="1.5" /><path d="M4 13 L10 3 M10 13 L16 3 M16 13 L21 5" stroke="var(--c-crit)" strokeWidth="1.3" /></Sw>}>✕ Blocked</Item>
      <Item sw={<Sw><rect x="5" y="2" width="12" height="12" rx="1.5" fill="var(--m-pallet)" /></Sw>}>Pallet</Item>
      <Item sw={<Sw><circle cx="11" cy="8" r="6" fill="#4aa3ff" /><path d="M16 8 L8.5 11.5 L8.5 4.5 Z" fill="var(--m-robot-stroke)" opacity=".75" /></Sw>}>Robot · heading</Item>
      <Item sw={<Sw><circle cx="11" cy="8" r="7" fill="none" stroke="var(--c-unknown)" strokeDasharray="2 1.5" /><circle cx="11" cy="8" r="4.5" fill="var(--c-unknown)" /></Sw>}>Stale robot</Item>
      <Item sw={<Sw><circle cx="11" cy="8" r="6" fill="none" stroke="var(--c-crit)" strokeWidth="2" /></Sw>}>In deadlock</Item>
      <Item sw={<Sw><path d="M12 2 L6 9 L10.5 9 L9 14 L16 6.5 L11.5 6.5 Z" fill="var(--c-accent)" /></Sw>}>Dock</Item>
      <Item sw={<Sw><path d="M11 2 V10 M7 7 L11 11 L15 7 M6 14 H16" stroke="var(--c-muted)" strokeWidth="1.6" fill="none" /></Sw>}>Drop</Item>
      <Item sw={<Sw><rect x="7" y="4" width="8" height="8" fill="none" stroke="var(--m-label)" strokeWidth="1.3" /></Sw>}>Pick · P bay</Item>
    </ul>
  );
}
