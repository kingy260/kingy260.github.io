# shayaan's website

A one-page academic site with a live, pushable Unitree G1 at the top. Plain HTML, CSS, and JavaScript: no build step, no frameworks, nothing to install.

```
index.html                  all the page text lives here
cv.pdf                      the CV linked from the page
assets/css/site.css         layout, plus the original red-and-grey palette
assets/css/motosu.css       the Motosu Lake palette (lake blue, aqua, wine red, cream, brass)
assets/js/g1.js             the interactive G1 (balance controller + leg IK)
assets/js/site.js           theme toggle, looping thumbnails, gallery viewer, starts the G1
assets/models/g1.json       the G1 model, simplified (about 150 KB to download)
assets/img/                 photo, thumbnails, poster, favicon
assets/img/full/            larger images that open in the gallery viewer
assets/logos/               CMU, TMU, MDA Space, Collins Aerospace logos
assets/video/               looping thumbnails (small) and full versions for the viewer
licenses/                   Unitree's BSD-3 license for the G1 model
```

## Put it online (GitHub Pages, free)

1. Sign in to GitHub as `kingy260`.
2. Create a new repository named exactly `kingy260.github.io`. Set it to **Public**. Don't add a README.
3. On the empty repository page, click **uploading an existing file**. Drag in everything inside this folder (the files and the `assets` and `licenses` folders, not the outer folder itself). Click **Commit changes**.
4. Go to **Settings → Pages**. Under "Build and deployment", set Source to **Deploy from a branch**, Branch to **main** and **/(root)**, then **Save**.
5. Wait a minute or two, then open `https://kingy260.github.io`.

To update later, open a file on GitHub, click the pencil icon, edit, and **Commit changes**. The live site refreshes within a minute or two.

## Change text yourself

1. On GitHub, open your repository and click `index.html`, then the pencil icon (Edit).
2. Press Ctrl+F (Cmd+F on a Mac) to find the words you want to change, and type over them.
3. Click **Commit changes**. The live site updates in about a minute.

Change only the words between the tags. In `<p class="desc">Worked on Canadarm3…</p>`, edit the sentence but leave `<p class="desc">` and `</p>` alone. Sections are marked with comments like `<!-- ==== INDUSTRY -->`, so search for those to jump around.

For a longer editing session, press the `.` key on your repository page to open a full editor in the browser (github.dev) and commit from its Source Control tab. GitHub keeps every saved version (the clock icon on a file shows its history), so you can always copy back an older one.

## Switch back to the original colors

In `index.html`, delete the one line that loads `assets/css/motosu.css` (it has a comment above it). Everything else stays the same.

## Pictures and videos

Every thumbnail opens a gallery viewer with arrows, swipe, and full screen. In `index.html`, each thumbnail block looks like this:

```html
<div class="thumb media">
  <a href="assets/img/full/big-photo.jpg" data-caption="What the photo shows"><img src="assets/img/projects/small-photo.jpg" alt="..."></a>
  <a href="assets/img/full/another.jpg" data-caption="Second picture in the gallery" hidden></a>
</div>
```

- The first link is what shows on the page. Extra links marked `hidden` are more pictures in the same gallery.
- For a video, point the link at an `.mp4` and add `data-type="video"` and `data-poster="..."` (a still image). The Helicopter Drilling entry is a full example with a looping video and three extra pictures.
- Thumbnails look best as 4:3 landscape crops around 480 × 360 pixels (JPG, under about 150 KB). Full-size images can be up to about 1600 pixels on the long side.
- Videos: MP4, 5–15 seconds. Keep the looping thumbnail small (under about 1 MB) and put the full-quality version in the gallery link.

## Research entries

The Research section has one entry per research position. A paper or talk goes in the "output" box inside the entry it came from. Research that hasn't produced a paper yet uses the same block with the output box left out.

## Optional: your own domain

Buy a domain (for example from Cloudflare or Namecheap, about $10–15 a year), then add it under **Settings → Pages → Custom domain** and follow GitHub's DNS instructions.

## Credits

Layout inspired by Jon Barron's website. Colors after the KEYKOBO Motosu Lake keycap set. G1 model © Unitree Robotics, BSD-3-Clause, from MuJoCo Menagerie, simplified for the web. Link icons from Font Awesome Free (CC BY 4.0). Licenses are in `licenses/`.
