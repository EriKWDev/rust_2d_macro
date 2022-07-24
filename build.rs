fn load_env_file(env_file: &str) {
    env_file.lines().for_each(|line| {
        let (key, value) = {
            let mut it = line.split('=');

            (it.next().unwrap(), it.next().unwrap())
        };

        println!("cargo:rustc-env={key}={value}");
    });
}

fn main() {
    let env_file = include_str!(".env");

    [env_file].into_iter().for_each(|file| {
        load_env_file(file);
    });
}
