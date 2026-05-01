# 🍦 Turkish Ice Cream

> Interactive kiosk-style demo stack — perception, motion, and HTTP API.

A lightweight **Turkish ice cream project** that combines a **FastAPI** core service, library packages for **hand tracking** and **motor / hardware adapters**, and a **Poetry + pyenv** workflow ready for Raspberry Pi or desktop development.

<!-- Optional: add screenshots under `./assets/` and uncomment.
<img src="./assets/banner.jpg" alt="Demo" width="100%">
-->

---

## ✨ Features

- blablabla

---

## 📋 Prerequisites

| Tool       | Version | Purpose                                |
| ---------- | ------- | -------------------------------------- |
| **Git**    | ≥ 2.30  | Clone the repository                   |
| **pyenv**  | ≥ 2.3   | Manage local Python versions           |
| **Python** | 3.12.x  | Runtime (pinned via `.python-version`) |
| **Poetry** | ≥ 2.1   | Dependencies & virtual environment     |

Optional: hardware targets (e.g. Raspberry Pi) depend on how you deploy `motor_adapter` — wire them in your integration layer, not required for running the API locally.

---

## 🚀 Quick Start

### 1 Clone the repository

```bash
git clone git@github.com:work-tomorrow/turkish-ice-cream.git
cd turkish-ice-cream
```

### 2 Install pyenv

<details>
<summary><b>macOS (Homebrew)</b></summary>

```bash
brew update
brew install pyenv
echo 'eval "$(pyenv init -)"' >> ~/.zshrc
source ~/.zshrc
```

</details>

<details>
<summary><b>Ubuntu / Debian</b></summary>

```bash
curl https://pyenv.run | bash
echo 'export PATH="$HOME/.pyenv/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init -)"' >> ~/.bashrc
source ~/.bashrc
```

</details>

### 3 Install & pin Python (matches `.python-version`)

```bash
pyenv install 3.12.3    # skip if already installed
pyenv local   3.12.3   # writes .python-version
python --version       # should print Python 3.12.x
```

### 4 Install Poetry

```bash
curl -sSL https://install.python-poetry.org | python3 -
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc   # or ~/.zshrc
source ~/.bashrc
poetry --version
```

### 5 Install dependencies

```bash
poetry env use "$(pyenv which python)"   # bind Poetry’s venv to pyenv’s Python
poetry install                           # creates .venv + installs packages
```

---

## 🛠 Common Commands

| Goal                      | Command                                                                    |
| ------------------------- | -------------------------------------------------------------------------- |
| Drop into virtual-env     | `eval "$(poetry env activate)"`                                            |
| Run core API (reload)     | `poetry run python -m core`                                                |
| Run core API (uvicorn)    | `poetry run uvicorn core.app.main:app --reload --host 0.0.0.0 --port 8000` |
| Open interactive API docs | Visit http://127.0.0.1:8000/docs after starting the server                 |

### Other Commands

| Goal                      | Command                            |
| ------------------------- | ---------------------------------- |
| Sync dependencies         | `poetry install`                   |
| Add a runtime dependency  | `poetry add <package>`             |
| Add a dev-only dependency | `poetry add --group dev <package>` |

---

## 📜 License

Distributed under the **MIT License**. See [`LICENSE`](LICENSE) for details.
