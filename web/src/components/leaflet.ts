// Leaflet base map shared by the site editor and the calibration tools: aerial imagery (default) and
// OpenStreetMap, zoomable past the imagery's native resolution for precise clicking.
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { useEffect, useRef, useState, type RefObject } from "react";

export function createBaseMap(el: HTMLElement, center: [number, number], zoom: number): L.Map {
  const map = L.map(el, { maxZoom: 23, zoomSnap: 0.25, wheelPxPerZoomLevel: 90 }).setView(center, zoom);
  const sat = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
    maxNativeZoom: 19,
    maxZoom: 23,
    attribution: "Imagery © Esri",
  }).addTo(map);
  const osm = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxNativeZoom: 19,
    maxZoom: 23,
    attribution: "© OpenStreetMap contributors",
  });
  L.control.layers({ Aerial: sat, Map: osm }, {}, { position: "topright" }).addTo(map);
  L.control.scale({ imperial: false }).addTo(map);
  return map;
}

// Map instance bound to a container's lifetime; re-measures when the container resizes.
export function useBaseMap(ref: RefObject<HTMLDivElement>, center: [number, number] | null, zoom: number) {
  const [map, setMap] = useState<L.Map | null>(null);
  const started = useRef(false);
  useEffect(() => {
    if (!ref.current || !center || started.current) return;
    started.current = true;
    const m = createBaseMap(ref.current, center, zoom);
    const ro = new ResizeObserver(() => m.invalidateSize());
    ro.observe(ref.current);
    setMap(m);
    return () => {
      ro.disconnect();
      m.remove();
      started.current = false;
      setMap(null);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ref, center?.[0], center?.[1]]);
  return map;
}

export const pin = (color: string, label: string | number, size = 18) =>
  L.divIcon({
    className: "",
    iconSize: [size, size],
    iconAnchor: [size / 2, size / 2],
    html: `<div style="width:${size}px;height:${size}px;border-radius:50%;background:${color};border:2px solid white;box-shadow:0 1px 4px rgba(0,0,0,.5);color:white;font:700 10px Inter,sans-serif;display:flex;align-items:center;justify-content:center">${label}</div>`,
  });
