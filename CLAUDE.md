# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

This repository is currently empty (no source files yet). It is intended to become a **single-page HTML/JS client-side telemetry dashboard** with no build step — the app is meant to run directly from a static HTML file in the browser.

Planned stack:
- **Plotly.js** — charting/visualization
- **PapaParse** — CSV parsing (loading telemetry data client-side)
- **Google GenAI SDK** — AI-assisted features (e.g. summarizing or querying telemetry data)

Since everything runs client-side in a single HTML page, there is no server, no bundler, and no package manager expected by default. If that changes (e.g. a build step or dependency manager is introduced), update this file with the actual commands.
