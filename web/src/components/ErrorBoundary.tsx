import { Component, type ReactNode } from "react";

// A failing view shows a message instead of blanking the whole panel.
export class ErrorBoundary extends Component<{ children: ReactNode; resetKey?: string }, { error: Error | null }> {
  state = { error: null as Error | null };
  static getDerivedStateFromError(error: Error) {
    return { error };
  }
  componentDidUpdate(prev: { resetKey?: string }) {
    if (prev.resetKey !== this.props.resetKey && this.state.error) this.setState({ error: null });
  }
  render() {
    if (!this.state.error) return this.props.children;
    return (
      <div className="flex h-full items-center justify-center p-6">
        <div className="card max-w-lg p-6 text-sm">
          <div className="mb-1 font-semibold">This view ran into a problem.</div>
          <div className="mb-4 break-words text-muted">{this.state.error.message}</div>
          <button className="btn-primary" onClick={() => this.setState({ error: null })}>
            Try again
          </button>
        </div>
      </div>
    );
  }
}
