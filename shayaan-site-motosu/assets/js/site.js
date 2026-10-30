/* Small page behaviours: theme toggle, looping thumbnails, the gallery viewer, and the G1 hero. */
(function () {
  'use strict';
  var root = document.documentElement;
  var reduceMotion = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  // ---- light / dark toggle (remembers the choice in this browser)
  var toggle = document.querySelector('[data-theme-toggle]');
  if (toggle) {
    toggle.addEventListener('click', function () {
      var current = root.getAttribute('data-theme');
      if (!current) current = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
      var next = current === 'dark' ? 'light' : 'dark';
      root.setAttribute('data-theme', next);
      try { localStorage.setItem('theme', next); } catch (e) { /* storage unavailable */ }
    });
  }

  // ---- thumbnails that loop while visible
  var vids = document.querySelectorAll('video[data-autoplay]');
  if (vids.length) {
    if (reduceMotion || !('IntersectionObserver' in window)) {
      vids.forEach(function (v) {
        v.addEventListener('mouseenter', function () { v.play().catch(function () {}); });
        v.addEventListener('mouseleave', function () { v.pause(); });
      });
    } else {
      var io = new IntersectionObserver(function (entries) {
        entries.forEach(function (en) {
          var v = en.target;
          if (en.isIntersecting) { v.preload = 'auto'; v.play().catch(function () {}); } else v.pause();
        });
      }, { threshold: 0.35 });
      vids.forEach(function (v) { io.observe(v); });
    }
  }

  // ---- gallery viewer: every .media block opens its links (images or videos) full size
  var ICON = {
    zoom: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M15 3h6v6M9 21H3v-6M21 3l-7 7M3 21l7-7"/></svg>',
    stack: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round" aria-hidden="true"><rect x="3" y="7" width="14" height="14" rx="2"/><path d="M7 3h12a2 2 0 0 1 2 2v12"/></svg>',
    close: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><path d="M6 6l12 12M18 6 6 18"/></svg>',
    full: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/></svg>',
    prev: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M15 5l-7 7 7 7"/></svg>',
    next: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M9 5l7 7-7 7"/></svg>',
  };
  var galleries = Array.prototype.slice.call(document.querySelectorAll('.media'));
  function itemsOf(box) {
    return Array.prototype.slice.call(box.querySelectorAll('a[href]')).map(function (a) {
      var href = a.getAttribute('href');
      var img = a.querySelector('img');
      var isVideo = a.getAttribute('data-type') === 'video' || /\.(mp4|webm)(\?|$)/i.test(href);
      return {
        href: href,
        video: isVideo,
        poster: a.getAttribute('data-poster') || '',
        thumb: a.getAttribute('data-thumb') || (isVideo ? a.getAttribute('data-poster') : href) || href,
        caption: a.getAttribute('data-caption') || (img && img.alt) || '',
      };
    });
  }
  galleries.forEach(function (box) {
    var n = box.querySelectorAll('a[href]').length;
    var zoom = document.createElement('span'); zoom.className = 'zoom'; zoom.innerHTML = ICON.zoom; box.appendChild(zoom);
    if (n > 1) {
      var more = document.createElement('span'); more.className = 'more'; more.innerHTML = ICON.stack + n; box.appendChild(more);
      more.setAttribute('aria-hidden', 'true');
    }
    var first = box.querySelector('a[href]');
    if (first) {
      first.setAttribute('aria-label', 'Open ' + (n > 1 ? 'gallery of ' + n + ' items' : 'full size') + ': ' + (itemsOf(box)[0].caption || 'media'));
      first.setAttribute('aria-haspopup', 'dialog');
    }
  });

  var dlg = null, list = [], at = 0, lastFocus = null;
  function buildViewer() {
    dlg = document.createElement('dialog');
    dlg.className = 'lightbox';
    dlg.setAttribute('aria-label', 'Media viewer');
    dlg.innerHTML =
      '<div class="lb-top"><span class="lb-count" aria-live="polite"></span><div class="lb-actions">' +
      '<button type="button" class="lb-btn lb-full" aria-label="Full screen">' + ICON.full + '</button>' +
      '<button type="button" class="lb-btn lb-close" aria-label="Close">' + ICON.close + '</button></div></div>' +
      '<div class="lb-stage"><button type="button" class="lb-btn lb-prev" aria-label="Previous">' + ICON.prev + '</button>' +
      '<div class="lb-media"></div>' +
      '<button type="button" class="lb-btn lb-next" aria-label="Next">' + ICON.next + '</button></div>' +
      '<p class="lb-caption"></p><div class="lb-strip"></div>';
    document.body.appendChild(dlg);
    dlg.querySelector('.lb-close').addEventListener('click', closeViewer);
    dlg.querySelector('.lb-prev').addEventListener('click', function () { show(at - 1); });
    dlg.querySelector('.lb-next').addEventListener('click', function () { show(at + 1); });
    var fullBtn = dlg.querySelector('.lb-full');
    if (!(document.fullscreenEnabled || document.webkitFullscreenEnabled)) fullBtn.hidden = true;
    fullBtn.addEventListener('click', function () {
      try {
        if (document.fullscreenElement) { document.exitFullscreen(); return; }
        var req = dlg.requestFullscreen || dlg.webkitRequestFullscreen;
        var p = req && req.call(dlg);
        if (p && p.catch) p.catch(function () { fullBtn.hidden = true; });
      } catch (e) { fullBtn.hidden = true; }
    });
    dlg.addEventListener('close', function () {
      root.classList.remove('lb-open');
      var v = dlg.querySelector('video'); if (v) v.pause();
      if (document.fullscreenElement) { try { document.exitFullscreen(); } catch (e) { /* ignore */ } }
      if (lastFocus && lastFocus.focus) lastFocus.focus();
    });
    // click on the dark area (not the media or controls) closes
    dlg.addEventListener('click', function (e) {
      if (e.target === dlg || e.target.classList.contains('lb-stage') || e.target.classList.contains('lb-media')) closeViewer();
    });
    dlg.addEventListener('keydown', function (e) {
      if (e.key === 'ArrowLeft') { e.preventDefault(); show(at - 1); }
      else if (e.key === 'ArrowRight') { e.preventDefault(); show(at + 1); }
    });
    // swipe between items on touch screens
    var sx = null, sy = null;
    dlg.addEventListener('pointerdown', function (e) { if (e.pointerType !== 'mouse') { sx = e.clientX; sy = e.clientY; } });
    dlg.addEventListener('pointerup', function (e) {
      if (sx === null) return;
      var dx = e.clientX - sx, dy = e.clientY - sy; sx = sy = null;
      if (Math.abs(dx) > 50 && Math.abs(dx) > Math.abs(dy) * 1.4) show(at + (dx < 0 ? 1 : -1));
    });
  }
  function show(i) {
    if (!list.length) return;
    at = (i + list.length) % list.length;
    var it = list[at], box = dlg.querySelector('.lb-media');
    var old = box.querySelector('video'); if (old) old.pause();
    box.innerHTML = '';
    var el;
    if (it.video) {
      el = document.createElement('video');
      el.src = it.href; el.controls = true; el.muted = true; el.loop = true; el.playsInline = true; el.autoplay = true;
      if (it.poster) el.poster = it.poster;
      el.setAttribute('aria-label', it.caption || 'Video');
    } else {
      el = document.createElement('img');
      el.src = it.href; el.alt = it.caption || '';
      el.decoding = 'async';
    }
    box.appendChild(el);
    if (it.video) { var pr = el.play(); if (pr && pr.catch) pr.catch(function () {}); }
    dlg.querySelector('.lb-caption').textContent = it.caption;
    var multi = list.length > 1;
    dlg.querySelector('.lb-count').textContent = multi ? (at + 1) + ' / ' + list.length : '';
    dlg.querySelector('.lb-prev').hidden = !multi;
    dlg.querySelector('.lb-next').hidden = !multi;
    var strip = dlg.querySelector('.lb-strip');
    Array.prototype.forEach.call(strip.children, function (b, k) { b.setAttribute('aria-current', k === at ? 'true' : 'false'); });
  }
  function openViewer(box, start) {
    if (!dlg) buildViewer();
    list = itemsOf(box);
    var strip = dlg.querySelector('.lb-strip');
    strip.innerHTML = '';
    strip.hidden = list.length < 2;
    list.forEach(function (it, k) {
      var b = document.createElement('button');
      b.type = 'button';
      b.setAttribute('aria-label', 'Show item ' + (k + 1) + (it.caption ? ': ' + it.caption : ''));
      var im = document.createElement('img'); im.src = it.thumb; im.alt = ''; im.loading = 'lazy';
      b.appendChild(im);
      b.addEventListener('click', function () { show(k); });
      strip.appendChild(b);
    });
    lastFocus = document.activeElement;
    root.classList.add('lb-open');
    if (dlg.showModal) dlg.showModal(); else dlg.setAttribute('open', '');
    show(start || 0);
    dlg.querySelector('.lb-close').focus();
  }
  function closeViewer() { if (dlg && dlg.open) dlg.close(); }
  galleries.forEach(function (box) {
    box.addEventListener('click', function (e) {
      var a = e.target.closest('a[href]');
      if (!a || !box.contains(a)) return;
      if (e.metaKey || e.ctrlKey || e.shiftKey) return; // let modified clicks open the file directly
      e.preventDefault();
      var links = Array.prototype.slice.call(box.querySelectorAll('a[href]'));
      openViewer(box, Math.max(0, links.indexOf(a)));
    });
  });

  // ---- interactive G1
  var fig = document.querySelector('[data-g1]');
  if (!fig) return;
  var stage = fig.querySelector('.g1-stage');
  var stats = fig.querySelector('[data-g1-stats]');
  var pushBtn = fig.querySelector('[data-g1-push]');
  var resetBtn = fig.querySelector('[data-g1-reset]');

  function webglOK() {
    try {
      var c = document.createElement('canvas');
      return !!(window.WebGLRenderingContext && (c.getContext('webgl2') || c.getContext('webgl')));
    } catch (e) { return false; }
  }
  function staticMode() { fig.classList.add('is-static'); }
  function plural(n, one, many) { return n + ' ' + (n === 1 ? one : many); }

  function boot() {
    if (!window.THREE || !window.G1Hero || !webglOK()) { staticMode(); return; }
    var api;
    window.G1Hero.mount(stage, {
      modelUrl: 'assets/models/g1.json',
      onStats: function (s) { if (stats) stats.textContent = plural(s.steps, 'step', 'steps') + ' · ' + plural(s.falls, 'fall', 'falls'); },
    }).then(function (a) {
      api = a;
      fig.classList.add('is-ready');
      window.G1 = api;
      if (pushBtn) pushBtn.addEventListener('click', function () { api.shove(); });
      if (resetBtn) resetBtn.addEventListener('click', function () { api.reset(); });
    }).catch(function () { staticMode(); });
    // mirror hover/grab state onto the figure so CSS can set the cursor
    new MutationObserver(function () {
      fig.classList.toggle('is-hover', stage.classList.contains('is-hover'));
      fig.classList.toggle('is-grabbing', stage.classList.contains('is-grabbing'));
    }).observe(stage, { attributes: true, attributeFilter: ['class'] });
  }

  if (document.readyState === 'complete') boot();
  else window.addEventListener('load', boot);
})();
