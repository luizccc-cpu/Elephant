fn main() {
    println!("cargo:rerun-if-changed=assets/elephant.ico");
    #[cfg(windows)]
    {
        let mut res = winresource::WindowsResource::new();
        res.set_icon("assets/elephant.ico");
        if let Err(e) = res.compile() {
            panic!("failed to compile Windows icon resource: {e}");
        }
    }
}
