---
name: aegis-git-publish
description: Commit, push or otherwise touch git/GitHub for A.E.G.I.S. (public repo yatharth1011/aegis-rescue-rover); also any request involving privacy, roll numbers, emails, submitted files, .gitignore, history, branches or repo settings.
---

# Git and the public repo

The repo `https://github.com/yatharth1011/aegis-rescue-rover` is **public**. Its history was rewritten once to remove private data. Don't undo that.

## Never in git
- Teammates' roll numbers. The pattern is `2026AIB` followed by 4 digits; they live only in the git-ignored `team_share/tools/team.json`.
- Personal email addresses and phone numbers.
- `to_submit/`, `course_templates/`, `FINAL/1_submission/` and `team_share/AEGIS_Proposal_Deck.pdf` (plus its FINAL copy).
- Build output, logs, datasets and calibration files (see `.gitignore`).

## Every commit
```bash
cd ~/Documents/Fusion/project
git status --short                                     # only the files you meant to change?
git add <specific files>                               # not "git add ." without reading the status
git diff --cached | grep -nE "2026[A-Z]{3}[0-9]{4}|@gmail|@iitrpr" || echo clean
git commit -m "One plain sentence: what changed and why"
git push origin main
```
- **No `Co-Authored-By:` or "Generated with ..." trailers for any AI tool.** No emojis in messages.
- The author email is the GitHub noreply address. Leave `git config user.email` alone.
- Commit the regenerated PDFs together with the source change that produced them.
- Ask before committing large binaries that are new; the repo is about 170 MB already.

## Ask first, always
Force-push, history rewrite (`git filter-repo`, `rebase` of pushed commits), deleting branches or tags, changing repo visibility or settings, adding collaborators, or touching `.gitignore` rules that protect private files.
