---
layout: default
title: Adding documentation
---

# Adding documentation

Our docs are hosted on GitHub pages using Jekyll and [Just the Docs](https://github.com/just-the-docs/just-the-docs). Documentation is written in markdown.

To add a new page to the docs, simply create a markdown file anywhere in the `docs/` folder of the `steward` repo, preferably under an appropriate subfolder (e.g. `docs/planning/`, `/docs/controls`). It's important that you add **front matter** to the top of the markdown file. A simple front matter section looks like this:

```
---
layout: default
title: Adding documentation
---
```

## Viewing/testing the website locally

See the official [Jekyll Quickstart](https://jekyllrb.com/docs/) guide, following steps 1-6 of the instructions there. Step 5 should be `cd steward/docs/`. Alternatively, you can install Docker, cd into `docs/`, and run:

```bash
docker run --rm --volume="$PWD:/srv/jekyll:Z" --publish [::1]:4000:4000 jekyll/jekyll jekyll serve
```

The non-Docker method is better supported, so try that one first.

## Deploying your changes

By design, the docs live inside the Steward git repo. To save your changes, simply commit them through this repo. Once they are pushed to GitHub, GitHub will build everything and deploy it automatically to the web.
