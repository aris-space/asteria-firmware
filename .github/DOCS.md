# HERMES CI workflows

In this contains the CI workflow configurations and the pull request template for the HERMES project.

This folder can be or was merged using a --allow-unrelated-histories merge
strategy from <https://github.com/aris-space/hermes-ci/>. By using this strategy
the history of the CI repo is included in your repo, and future updates to
the CI can be merged later.

To perform this merge run:

```shell
git remote add ci git@github.com:aris-space/hermes-ci.git
git fetch ci
git merge --allow-unrelated-histories ci/main
```

The repository and all its submodules must be added to the PAT token `HERMES_CI_TOKEN`, contact @Indeximal.

Sources: 
- <https://www.youtube.com/watch?v=xUH-4y92jPg&t=491s&ab_channel=JonGjengset> (05.10.2024) 
- <https://github.com/jonhoo/rust-ci-conf/tree/main> (05.10.2024)
- <https://github.com/actions/checkout/issues/116#issuecomment-644419389> (13.2.2025)
