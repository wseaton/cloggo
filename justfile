prune-deps:
    cargo udeps --output json | jq -r '.unused_deps."cloggo 0.1.0 (path+file:///home/weaton/git/cloggo)".normal' | jq -r '.[]' | xargs -n1 cargo rm