# Codex
Team 4201's Generic Robot Software Library

## Development Notes
* Maven artifacts are saved under the [releases/](releases) directory after running `./gradlew publish`.
* To publish changes to the library, run `./gradlew publish` and push the updated files under `releases/` to GitHub.
  * TODO: This can be automated with a GitHub action. 
* The Codex repo is set to host the root directory of the main branch as a GitHub page, which allows the vendordep JSON file to pull the library.