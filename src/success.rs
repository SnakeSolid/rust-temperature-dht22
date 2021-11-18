pub trait Success<T> {
    fn success(self) -> T;
}

impl<T, E> Success<T> for Result<T, E> {
    #[inline]
    fn success(self) -> T {
        match self {
            Ok(value) => value,
            Err(_) => unreachable!(),
        }
    }
}
