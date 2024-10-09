# Dev Environment Setup

1. Install `pipx` following your OS instructions from [Github page](https://github.com/pypa/pipx)

2. Fork the [Github repository](https://github.com/ayushbaid/oa-drone-trajectory-oct-2024) and clone it on your computer.

3. Install `poetry`.
    
    ```pipx install poetry```
4. Install the dependencies for this repository.

    ```poetry install```

4. Find out the venv and configure your editor to use it.

    ``` poetry env info --path```

5. If using a terminal, you can use `poetry shell`.

6. Now you can run all python commands in the terminal.

7. Commit your `poetry.lock` file to your repo.