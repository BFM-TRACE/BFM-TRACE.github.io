const $ = (selector, root = document) => root.querySelector(selector);
const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];
const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
const hero = $("#hero-video");
const heroPause = $("#hero-pause");
let heroManuallyPaused = reduceMotion.matches;
if (reduceMotion.matches) hero.pause();
function updateHeroControl() {
  heroPause.textContent = hero.paused ? "▶" : "Ⅱ";
  heroPause.setAttribute(
    "aria-label",
    hero.paused ? "Play background video" : "Pause background video",
  );
}
hero.addEventListener("play", updateHeroControl);
hero.addEventListener("pause", updateHeroControl);
heroPause.addEventListener("click", () => {
  heroManuallyPaused = !hero.paused;
  if (hero.paused) {
    hero.style.display = "block";
    hero.play().catch(() => {});
  } else hero.pause();
});
new IntersectionObserver(
  ([entry]) => {
    if (!entry.isIntersecting) hero.pause();
    else if (!heroManuallyPaused) hero.play().catch(() => {});
  },
  { threshold: 0.05 },
).observe(hero);
updateHeroControl();

// Defer the renderer, meshes and simulation connection until the demo is near view.
const frame = $("#demo-frame");
let demoSizeObserver;
frame.addEventListener("load", () => {
  demoSizeObserver?.disconnect();
  const content = frame.contentDocument?.querySelector(".demo");
  if (!content) return;
  demoSizeObserver = new ResizeObserver(() => {
    frame.style.height = `${Math.ceil(content.getBoundingClientRect().height) + 2}px`;
  });
  demoSizeObserver.observe(content);
});
const frameObserver = new IntersectionObserver(
  ([entry]) => {
    if (entry.isIntersecting && !frame.getAttribute("src")) {
      frame.src = frame.dataset.src;
      frameObserver.disconnect();
    }
  },
  { rootMargin: "350px" },
);
frameObserver.observe(frame);

const gallery = $("#gallery");
const automaticPauses = new WeakSet();
function pauseAutomatically(video) {
  if (!video.paused) {
    automaticPauses.add(video);
    video.pause();
  }
}
const tabs = $$(".gallery-tabs [role=tab]");
const descriptions = {
  motion: "Dynamic whole-body reference tracking on the full-scale Unitree H2.",
  teleoperation:
    "Human-guided whole-body interaction through VR teleoperation.",
  vla: "Autonomous task demonstrations using the transferred low-level controller.",
};
const mediaObserver = new IntersectionObserver(
  (entries) => {
    for (const { target, isIntersecting } of entries) {
      if (
        isIntersecting &&
        !reduceMotion.matches &&
        !target.dataset.userPaused
      ) {
        if (!target.src && target.dataset.src) {
          target.src = target.dataset.src;
          target.load();
        }
        target.play().catch(() => {});
      } else pauseAutomatically(target);
    }
  },
  { threshold: 0.25 },
);
let items = [];
function renderCategory(category) {
  for (const video of $$("video", gallery)) {
    video.pause();
    mediaObserver.unobserve(video);
    video.removeAttribute("src");
    video.load();
  }
  gallery.replaceChildren();
  for (const tab of tabs) {
    const selected = tab.dataset.category === category;
    tab.setAttribute("aria-selected", String(selected));
    tab.tabIndex = selected ? 0 : -1;
  }
  gallery.setAttribute("aria-labelledby", `tab-${category}`);
  $("#gallery-description").textContent = descriptions[category];
  for (const item of items.filter((item) => item.category === category)) {
    const article = document.createElement("article");
    article.className = "video-card";
    const wrap = document.createElement("div");
    wrap.className = "video-wrap";
    const video = document.createElement("video");
    video.controls = true;
    video.muted = true;
    video.defaultMuted = true;
    video.loop = true;
    video.playsInline = true;
    video.preload = "none";
    video.poster = item.poster;
    video.src = item.src;
    video.setAttribute("aria-label", `${item.title} on Unitree H2`);
    video.addEventListener("pause", () => {
      if (automaticPauses.has(video)) automaticPauses.delete(video);
      else video.dataset.userPaused = "true";
    });
    video.addEventListener("play", () => {
      delete video.dataset.userPaused;
    });
    video.addEventListener(
      "error",
      () => {
        if (!video.error) return;
        const error = document.createElement("p");
        error.className = "video-error";
        error.textContent = "This video could not be loaded. ";
        const link = document.createElement("a");
        link.href = item.src;
        link.textContent = "Open the clip";
        error.append(link);
        wrap.replaceChildren(error);
      },
      { once: true },
    );
    wrap.append(video);
    const meta = document.createElement("div");
    meta.className = "video-meta";
    const title = document.createElement("h3");
    title.textContent = item.title;
    const duration = document.createElement("span");
    duration.textContent = `${Math.round(item.duration || 0)} sec · 1×`;
    meta.append(title, duration);
    article.append(wrap, meta);
    if (item.description) {
      const desc = document.createElement("p");
      desc.className = "video-description";
      desc.textContent = item.description;
      article.append(desc);
    }
    gallery.append(article);
    mediaObserver.observe(video);
  }
}
tabs.forEach((tab, index) => {
  tab.addEventListener("click", () => renderCategory(tab.dataset.category));
  tab.addEventListener("keydown", (event) => {
    let next;
    if (event.key === "ArrowRight") next = (index + 1) % tabs.length;
    if (event.key === "ArrowLeft")
      next = (index - 1 + tabs.length) % tabs.length;
    if (event.key === "Home") next = 0;
    if (event.key === "End") next = tabs.length - 1;
    if (next !== undefined) {
      event.preventDefault();
      tabs[next].focus();
      renderCategory(tabs[next].dataset.category);
    }
  });
});
try {
  const response = await fetch("content/media.json");
  if (!response.ok) throw new Error("Media index is unavailable.");
  const data = await response.json();
  items = data.items;
  renderCategory("motion");
} catch {
  gallery.replaceChildren();
  const message = document.createElement("p");
  message.className = "loading-message";
  message.textContent =
    "Demonstrations could not be loaded. Please reload the page or watch the overview video above.";
  gallery.append(message);
}

// Avoid several off-screen decoders running while the page is in the background.
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    hero.pause();
    for (const video of $$("video", gallery)) pauseAutomatically(video);
  } else {
    if (!heroManuallyPaused && hero.getBoundingClientRect().bottom > 0)
      hero.play().catch(() => {});
    for (const video of $$("video", gallery)) {
      const r = video.getBoundingClientRect();
      if (
        r.top < innerHeight &&
        r.bottom > 0 &&
        !reduceMotion.matches &&
        !video.dataset.userPaused
      )
        video.play().catch(() => {});
    }
  }
});
