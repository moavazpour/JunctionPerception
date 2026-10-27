import { useEffect, useState } from "react";
import { Header } from "./components/Header";
import { LiveView } from "./components/LiveView";
import { SettingsPage } from "./settings/SettingsPage";
import { resolvedTheme, useStore } from "./store";

export interface Route {
  page: "live" | "settings";
  section: string;
}

function parse(): Route {
  const h = location.hash.replace(/^#\/?/, "").split("/");
  if (h[0] === "settings") return { page: "settings", section: h[1] || "calibration" };
  return { page: "live", section: "" };
}

export function navigate(path: string) {
  location.hash = `#/${path}`;
}

function useHashRoute() {
  const [r, setR] = useState(parse);
  useEffect(() => {
    const f = () => setR(parse());
    window.addEventListener("hashchange", f);
    return () => window.removeEventListener("hashchange", f);
  }, []);
  return r;
}

// Applies the theme to <html data-theme> and follows the system setting live.
function useTheme() {
  const pref = useStore((s) => s.theme);
  const [sys, setSys] = useState(0);
  useEffect(() => {
    const mq = window.matchMedia?.("(prefers-color-scheme: dark)");
    const f = () => setSys((x) => x + 1);
    mq?.addEventListener("change", f);
    return () => mq?.removeEventListener("change", f);
  }, []);
  const t = resolvedTheme(pref);
  useEffect(() => {
    document.documentElement.dataset.theme = t;
  }, [t, sys]);
  return t;
}

export function App() {
  const route = useHashRoute();
  useTheme();
  return (
    <div className="flex h-full flex-col">
      <Header route={route} />
      <main className="min-h-0 flex-1">{route.page === "settings" ? <SettingsPage section={route.section} /> : <LiveView />}</main>
    </div>
  );
}
