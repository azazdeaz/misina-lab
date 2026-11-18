# Writing the docs

How a page in this folder is written, and how it stays true as the code
changes under it. It is for anyone editing the docs, person or agent; the
`CLAUDE.md` beside it loads it for Claude Code whenever a page here is
opened. `tests/test_docs.py`, at the repository root, checks what a regex
can, and this guide covers the rest.

## Who reads a page

A person who wants to build a scene generator, or to understand one they
use. They write Rust and Python and have used a simulator such as Isaac Lab,
but they have not read this code, and may know Bevy only from the README's
example. Many arrive from a search or a link, not from the page before, and
English is a second language for many of them.

So a page is an introduction: what a thing is, what it is for and how it
fits with the rest, in plain words. It runs from the definition to one
example to the detail, with the source one link away, and a reader stops
once they have what they came for. A page is not written for an agent
working on the code, which has the `AGENTS.md` maps and the module docs,
and reads a page written for a person without trouble.

## What goes where

| Place | What it holds |
| --- | --- |
| a module doc or a docstring | that module's contract: what it is, what it promises, the rules for using it, and why it is built the way it is |
| a page in this folder | what an idea is and what it is for, how the modules fit together, and what someone building a generator has to know to use them |
| [`../README.md`](../README.md) | the front door: what the crate is, the smallest generator, and the table of pages |
| [`../AGENTS.md`](../AGENTS.md) | the map: where things are, and the rules that bite |

Each fact is written once and linked from everywhere else. A sentence that
would fit both a page and a module doc goes in the module doc, where it is
read beside the code it describes and changes with it. Why the framework's
own code is built the way it is goes there too; a page keeps what a reader
needs to use it.

## Writing a page

1. Title it with a noun naming the idea as a reader would ask for it,
   `# Mesh variations`, not after the technique or the module behind it. A
   how-to is titled with what it does, `# Editing parameters`.
2. Define the idea in the first paragraph, and say what it is for, in words
   a newcomer follows ([Prose](#prose)).
3. Show one example. Code is the row of boxes from the README, or a generic
   plant, leaf or stem, trimmed with `/* ... */` to what the point needs.
4. Give each further section a heading that names what it explains, so a
   link can land on it.
5. End with `## Where it lives`: the files, each with the symbols a reader
   would open it for. A how-to has numbered steps in place of the sections,
   and links each file where a step uses it.
6. Add a row to the README's table. The test fails while a page is missing
   from it.

One idea per page. Split a page when it takes on a second idea or passes
about 200 lines.

## Prose

- State a fact about the code or a rule to follow, not how it was found, and
  not the answer to a question only ever asked in a chat: a design that was
  considered and dropped is not on the page. If a detail doesn't change what
  the reader does next, cut it.
- A page stands on its own. The first time it uses a term from elsewhere, a
  fragment, the panel, the snippet, it says what that is in a few words and
  links the page that defines it. The full definition stays on that page.
  "The panel" cannot be a page's first mention of the panel.
- Say what happens before how. State the effect in words a newcomer follows,
  "an edit rebuilds only what depends on it", and the mechanism after it.
  What a generator's own code has to do with Bevy, PyO3 or USD to get that
  effect goes in a `> [!NOTE]` right after the paragraph it explains,
  starting **In Bevy terms.**, **In PyO3 terms.** or **In USD terms.** A
  page reads complete without its notes.
- Use plain words and short sentences. Replace an idiom or a figure of
  speech by what it means: "the framework is keyed on this type" by "the
  framework is given this type", "is a function of it" by "is made from it",
  "a budget buys variety" by "a larger budget gives more variety". A term of
  art, k-center, a codebook, gets a few words of explanation at first use.
- Give the reason for a rule that isn't obvious. A rule without one is
  broken by the first reader who thinks they know better.
- Say what a measured number was measured on.
- Name no generator. The framework's examples are its own, and the test
  fails on a generator's name or vocabulary anywhere in this crate or its
  Python package.

## Links

- Link the file and name the symbol: [`src/rng.rs`](../src/rng.rs), `salt`.
  Never link a line number. Lines move with every edit and nothing notices,
  while a symbol survives and can be grepped. The test fails on a line link.
- Link a heading, `elements.md#rules`, to land a reader on one section. The
  test checks that the heading exists.
- Keep links relative and inside the repository. The test fails on one that
  resolves to nothing.

## Markdown that renders everywhere

A page is read on GitHub, in an editor and on a rendered site; the
README is read through rustdoc as well. Write what all of them agree on: one
H1, fenced code with a language tag, tables, Mermaid for diagrams. No raw
HTML, and no renderer-specific callouts such as `!!! note`. GitHub's alerts
are the exception, since GitHub and the site draw them as boxes and an
editor as a blockquote. They have two uses: the `> [!WARNING]` under a
page's title, which says the page has not been checked against the current
source and is removed once it has, and the `> [!NOTE]` that holds what a
generator's own code has to do with Bevy, PyO3 or USD ([Prose](#prose)).

## Checking a page

A writer cannot see what their own page leaves out, so a new or rewritten
page is read once by someone who knows nothing of the project before it is
done. For an agent that is a second agent, told to play the reader of
[Who reads a page](#who-reads-a-page), to read that page and no other file,
and to disregard what it was told about the repository. It lists every
sentence where it stops: a term the page has not explained, a phrase that
has to be decoded, a sentence too dense for one read.

Fix what it finds, then check each rewritten claim against the source
again. A sentence made simpler is often made less accurate.

## Keeping a page current

A page goes stale when the code changes under it, and no test reads prose.
So the change that renames a public item, or changes what one does, also
fixes the pages that name it:

1. Grep this folder and the README for the old name.
2. Fix what each hit says, not only the name.
3. Delete a page whose idea is gone, with its row in the README's table.

A renamed heading breaks the links into it, and the test names them. The
README's Rust block is a doctest, so `cargo test` catches it going stale.
Nothing else here is compiled: a code block on a page is a sketch, read
against the code it sketches.
