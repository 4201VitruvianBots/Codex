# Codex
Team 4201's Generic Robot Software Library

## Versioning
_Note: Not a hard rule at the moment, but just my thoughts_

Versioning uses the following numbering pattern **YYYY.AA.BB**
* **YYYY** will always be the FRC year the library is compatible with
* **AA** will be the week in the FRC Season the change was made.
  * TODO: What about off-season? Should event changes have their own marking/tag?
* **BB** will just be an increment from the previous published version.

## Testing
* Unit tests can be tested by running `./gradlew test`
* Testing of GUI elements can be tested using `./gradlew simulateJava`

## Development Notes
* Maven artifacts are saved under the [releases/](releases) directory after running `./gradlew publish`.
* To publish changes to the library, run `./gradlew publish` and push the updated files under `releases/` to GitHub.
  * TODO: This can be automated with a GitHub action.
* The Codex repo is set to host the root directory of the main branch as a GitHub page, which allows the vendordep JSON file to pull the library.
