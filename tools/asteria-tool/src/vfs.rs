use anyhow::{Result, bail};

pub fn normalize_path(cwd: &str, input: &str) -> Result<String> {
    let cwd = if cwd.is_empty() { "/" } else { cwd };
    if !cwd.starts_with('/') {
        bail!("invalid cwd: must be absolute");
    }

    let mut parts: Vec<&str> = Vec::new();
    if !input.starts_with('/') {
        for part in cwd.split('/').filter(|p| !p.is_empty()) {
            parts.push(part);
        }
    }

    for part in input.split('/') {
        match part {
            "" | "." => {}
            ".." => {
                let _ = parts.pop();
            }
            other => parts.push(other),
        }
    }

    if parts.is_empty() {
        Ok("/".to_string())
    } else {
        Ok(format!("/{}", parts.join("/")))
    }
}

pub fn file_name(path: &str) -> Result<&str> {
    if path == "/" {
        bail!("path has no file name: /");
    }

    path.rsplit('/')
        .find(|s| !s.is_empty())
        .ok_or_else(|| anyhow::anyhow!("path has no file name: {path}"))
}

pub fn join(parent: &str, child: &str) -> Result<String> {
    if child.contains('/') || child.is_empty() {
        bail!("invalid child path segment: {child}");
    }
    if parent == "/" {
        return Ok(format!("/{child}"));
    }
    Ok(format!("{parent}/{child}"))
}
