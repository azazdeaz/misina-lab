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

That reader sets what a page may assume. Rust and Python, a terminal, git
and JSON need no explaining, and neither does what any user of a simulator
has met: a mesh, a rigid body, a joint, a collider, a quaternion. Everything
else is explained or linked ([Prose](#prose)).

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
6. Add a row to the README's table and an entry to the nav in
   `zensical.toml`, each under the group the page belongs to. The test fails
   while a page is missing from either.

One idea per page. Split a page when it takes on a second idea: a part that
answers a question of its own, which a reader would come for without the
rest, and which stands alone after a sentence or two of setup. Length alone
is no reason to split. Past about 300 lines, look again for such a part. An
explanation in which each section needs the one before it stays whole, and
so does a how-to that is followed from top to bottom.

## Prose

- State a fact about the code or a rule to follow, not how it was found, and
  not the answer to a question only ever asked in a chat: a design that was
  considered and dropped is not on the page. If a detail doesn't change what
  the reader does next, cut it.
- A page stands on its own, without teaching what belongs elsewhere. How
  much it says about a term depends on whose term it is:
  - **Ours**, a word this framework gave its meaning: a fragment, the panel,
    the snippet, a rod. The first time a page uses it, the page says what it
    is in a few words and links the page that defines it. The full
    definition stays on that page. "The panel" cannot be a page's first
    mention of the panel.
  - **Another project's**: a Bevy system, a USD prim, an Isaac Lab scene
    config, a Newton solver. The page says in a few words what the sentence
    needs, links that project's documentation at the first use
    ([Links](#links)), and teaches no more of it.
  - **The reader's own** ([Who reads a page](#who-reads-a-page)): no
    explanation.

  When it is unclear whether a term is another project's or the reader's
  own, link it and explain nothing.
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
- Link a file of this repository by its relative path. The test fails on
  one that resolves to nothing. A URL is for what a relative path cannot
  reach: another project's documentation, or this repository at a
  release's tag.
- Link another project's documentation where a page first leans on one of
  its terms: the page for that term, not the project's front page, at a URL
  that follows the project's latest version. No test reads such a URL, so
  check that it loads, and that its anchor exists, when you add it.
- For what a public Rust item takes and returns, link its page on docs.rs,
  `https://docs.rs/misina-lab/latest/misina_lab/...`, and leave the detail
  there. docs.rs shows the last release, so an item that no release has yet
  gets no link. The test checks each such link against the source: its
  module must be a file of the crate, and that file must name the item.

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

The reader's list shows where to look. It is not a score: a reader always
stops somewhere, so the list stays about as long however good the page
gets, and answering every stop makes the page longer, not better. Fix a
stop when the term is ours and the page has not said what it is, when the
term is another project's and has neither a few words nor a link, or when
the reader would do the wrong thing without the answer. Leave a stop that
only asks for more about something the reader can look up or already knows
([Prose](#prose)).

After fixing, check each rewritten claim against the source again. A
sentence made simpler is often made less accurate.

## Keeping a page current

A page goes stale when the code changes under it, and no test reads prose.
So the change that renames a public item, or changes what one does, also
fixes the pages that name it:

1. Grep this folder and the README for the old name.
2. Fix what each hit says, not only the name.
3. Delete a page whose idea is gone, with its row in the README's table and
   its entry in the nav.

A renamed heading breaks the links into it, and the test names them. The
README's Rust block is a doctest, so `cargo test` catches it going stale.
Nothing else here is compiled: a code block on a page is a sketch, read
against the code it sketches.
