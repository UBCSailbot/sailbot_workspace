import { NextRequest, NextResponse } from 'next/server';

export function middleware(request: NextRequest) {
  // if api is disabled, intercept all /api calls and return empty data
  if (
    process.env.DISABLE_API === 'true' &&
    request.nextUrl.pathname.startsWith('/api/')
  ) {
    return NextResponse.json({ success: true, data: [] });
  }
}

export const config = {
  matcher: '/api/:path*',
};
